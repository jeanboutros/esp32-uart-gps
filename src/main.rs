use std::ffi::{c_void, CStr, CString};
use std::ops::Deref;
/*
GPIOs not recommended to be used:
6 - 11, 16 - 17
src: https://docs.rs/crate/esp-idf-hal/latest
 */
use enumset::*;
use std::ptr::{null, null_mut};
use esp_idf_hal::cpu::Core;
use esp_idf_hal::delay::NON_BLOCK;
use esp_idf_svc::sys::SOC_UART_FIFO_LEN;
use esp_idf_svc::hal::uart::*;
use esp_idf_svc::hal::uart::config::{EventConfig, EventFlags};
use esp_idf_svc::hal::{
    prelude::*,
    peripherals::Peripherals,
    gpio::*,
    interrupt::InterruptType,
    delay::FreeRtos,
    task::*
};

const GPS_READ_BUFFER_SIZE: usize = 1024;

#[repr(C)] 
struct UartDriverDetails<'a> {
    driver: &'a UartDriver<'a>,
}

fn process_gps_message(message: &str) {
    log::info!("**** Full message recived... Processing {}", message);
}

fn concatenate_uart_rx_messages(latest_uart_string: &String, leftover_uart_string: &String) -> String {
    let binding = String::from_iter(vec![leftover_uart_string.clone(), latest_uart_string.clone()]);
    let split_messages = binding.split_terminator("\r\n");
    let new_latest_uart_string: Vec<&str> = split_messages.collect();
    

    // new_latest_uart_string.iter().for_each(|&_s| log::info!(">>{}", _s));
        
    let last = new_latest_uart_string.iter().last().unwrap();
    let lef = {
        match latest_uart_string.ends_with("\r\n") {
            true => "",
            false => last
        }
    };

    if !lef.is_empty() {
        log::debug!(">SKIPPING LEFTOVER STRING {}", &lef);
    }

    let num_messages_to_process = new_latest_uart_string.len() - (if lef.is_empty() {0} else {1});
    log::debug!("We have {} messages in the vector. We will process {} of them.", new_latest_uart_string.len(), num_messages_to_process);

    // keep this until the end??
    new_latest_uart_string
        .iter()
        .take(num_messages_to_process)
        .into_iter()
        .for_each(|message| process_gps_message(message));
    
    return lef.to_owned()
}
fn read_uart_rx_buffer<'a>(uart_driver: &'a UartDriver<'a>, read_buf: &'a mut [u8; GPS_READ_BUFFER_SIZE], last_leftover_string: &String) -> Result<String, &'static str>{
    let read_bytes = uart_driver.remaining_read().unwrap();
    if read_bytes < 1 {
        log::info!("Nothing to read. UART buffer's remaining_read is {}.", read_bytes);
        return Err("Empty buffer");
    }

                        
    match uart_driver.read(read_buf, NON_BLOCK) {
        Ok(bytes_read) => {
            let s = read_buf[..bytes_read].iter()
                                    .filter(|&b| *b >= 32 && *b <= 126 || ACCEPTABLE_CHARS.contains(b))  // Filter for printable ASCII characters (32-126)
                                    .map(|&b| b as char)  // Convert bytes to chars
                                    .collect::<String>();
            log::debug!("Received:\n>>>>{}<<<<", s);

            // let leftover_string = concatenate_uart_rx_messages(&s, &last_leftover_string);
            // return Ok(leftover_string);

            return Ok(s);
        },
        Err(error) => {
            log::error!("{}", error);
            return Err("Failed to read bytes.");
        }
        
    }
}

extern "C" fn uart_gps_task_handler(args: *mut c_void) {
    let uart_gps_driver_details = unsafe {
        // Safely create a reference from the raw pointer
        &*(args as *const UartDriverDetails)
    };

    let uart_driver = uart_gps_driver_details.driver;

    // log::info!("uart_gps_driver_details: {}", uart_driver.baudrate().unwrap());

    let uart_event_queue = uart_driver.event_queue().unwrap();
    let mut read_buf: [u8; GPS_READ_BUFFER_SIZE] = [0; GPS_READ_BUFFER_SIZE];
    let mut leftover_string: String = String::new();
    
    let _ = uart_driver.clear_rx();
    
    loop {
        
        match uart_event_queue.recv_front(200) {
            None => {FreeRtos::delay_ms(100)},
            Some((uart_event, _)) => {
                match uart_event.payload() {
                   
                    UartEventPayload::RxFifoOverflow | UartEventPayload::RxBufferFull => {
                        let _ = uart_driver.clear_rx();
                        log::debug!("Clearing the buffer!");
                    },
                    UartEventPayload::Data { size, timeout } => {
                        log::debug!("Received data with size {} bytes and timeout {}ms", size, timeout);
                        // let last_leftover_string = read_uart_rx_buffer(&uart_driver, &mut read_buf, &leftover_string).unwrap();
                        // leftover_string = last_leftover_string;
                        let received_string = read_uart_rx_buffer(&uart_driver, &mut read_buf, &leftover_string).unwrap();
                        // leftover_s
                        leftover_string = concatenate_uart_rx_messages(&received_string, &leftover_string);
                    },
                    UartEventPayload::PatternDetected | UartEventPayload::Break  => {
                        log::debug!("Looks like we have a break here!");
                        // Not handled yet
                    }
                    _ => {
                        log::debug!("Received uart_event {:?}", uart_event.payload());
                        // Not handled yet
                    }
                }
            }    
        }
    }


    unsafe { destroy(std::ptr::null_mut()) };
}

const UART_GPS_TASK_NAME: &CStr = unsafe { CStr::from_bytes_with_nul_unchecked(b"UART GPS TASK\0") };
const UART_GPS_STACK_SIZE: usize = 1024 * 4;
const UART_GPS_TASK_PRIORITY: u8 = 1;
// const UART_GPS_PORT = esp_idf_hal::uart::UART1.clone();
const ACCEPTABLE_CHARS: [u8; 2] = [b'\r', b'\n'];


fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Starting");
    
    let peripherals = Peripherals::take().unwrap();
    let uart_port = peripherals.uart2;
    let tx = peripherals.pins.gpio17;
    let rx = peripherals.pins.gpio18;
    let cts = Option::<AnyIOPin>::None;
    let rts = Option::<AnyIOPin>::None;
    
    let uart_gps_config = UartConfig {
        
        baudrate: Hertz(9_600),
        rx_fifo_size: (SOC_UART_FIFO_LEN * 8) as usize,
        ..Default::default()
    };

    let uart_gps_driver = UartDriver::new(uart_port, tx, rx, cts, rts, &uart_gps_config).unwrap();
    log::info!("UartDriver Created");

    // let mut read_buf: [u8; GPS_READ_BUFFER_SIZE] = [0; GPS_READ_BUFFER_SIZE];
    // loop {
    //     match uart_gps_driver.read(&mut read_buf, NON_BLOCK) {
    //         Ok(bytes_read) => {
    //             let s = read_buf.iter()
    //             .filter(|&b| *b >= 32 && *b <= 126 || ACCEPTABLE_CHARS.contains(b))  // Filter for printable ASCII characters (32-126)
    //             .map(|&b| b as char)  // Convert bytes to chars
    //             .collect::<String>();

    //             log::info!("Received: {}", s);
    //         },
    //         Err(error) => {log::error!("{}", error);}
            
    //     };
    //     FreeRtos::delay_ms(10);
    // }
    
    log::info!("UartDriver done reading");

    let mut uart_gps_driver_details = UartDriverDetails { driver: &uart_gps_driver };

    // uart_gps_driver.
    let q = uart_gps_driver.event_queue().unwrap();
    
    
    unsafe { 
        let _ = esp_idf_svc::hal::task::create(uart_gps_task_handler, 
                                                UART_GPS_TASK_NAME, 
                                                UART_GPS_STACK_SIZE, 
                                                &mut uart_gps_driver_details as *mut _ as *mut c_void,
                                                UART_GPS_TASK_PRIORITY,
                                            Option::<Core>::Some(Core::Core0)); 
    };

    loop {
        log::info!("Loopin ZZ main..");
        FreeRtos::delay_ms(1000);
    }
    
}
