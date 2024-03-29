

pub mod nmea {
    
    pub mod message {
        #[derive(Debug)]
        pub enum NmeaMessageTypeCode {
            GSA,
            GGA,
            GSV,
            NotImplemented,
        }
        #[derive(Debug)]
        pub enum GpsQualityIndicator {
            FixNotAvailable = 0,
            GpsFix = 1,
            DifferentialGpsFix = 2,
            PpsFix = 3,
            RealTimeKinematic = 4,
            FloatRtk = 5,
            Estimated = 6,
            ManualInputMode = 7,
            SimulationMode = 8,
            Invalid,
        }
        impl From<u8> for GpsQualityIndicator {
            fn from(value: u8) -> Self {
                match value {
                    0 => GpsQualityIndicator::FixNotAvailable,
                    1 => GpsQualityIndicator::GpsFix,
                    2 => GpsQualityIndicator::DifferentialGpsFix,
                    3 => GpsQualityIndicator::PpsFix,
                    4 => GpsQualityIndicator::RealTimeKinematic,
                    5 => GpsQualityIndicator::FloatRtk,
                    6 => GpsQualityIndicator::Estimated,
                    7 => GpsQualityIndicator::ManualInputMode,
                    8 => GpsQualityIndicator::SimulationMode,
                    _ => GpsQualityIndicator::Invalid,
                }
            }
        }
        
        pub trait NmeaMessage {
            fn parse(message: &str) -> Self;
            fn get_checksum(&self) -> String;
            fn get_talker_id(&self) -> String;
            fn get_message_type_code(&self) -> &NmeaMessageTypeCode;
        }
        #[derive(Debug)]
        pub struct UTCTime {
            hhmmss_ss: String,
        }

        impl UTCTime {
            fn new(_hhmmss_ss: &str) -> Self {
                UTCTime { hhmmss_ss: _hhmmss_ss.to_string() }
            }
        }

        
        //                                                       11
        //         1         2       3 4        5 6 7  8   9  10 |  12 13  14   15
        //         |         |       | |        | | |  |   |   | |   | |   |    |
        // $--GGA,hhmmss.ss,ddmm.mm,a,ddmm.mm,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh<CR><LF>
        // Ref: https://gpsd.gitlab.io/gpsd/NMEA.html#_gga_global_positioning_system_fix_data
        // Examples:
        // $GNGGA,155338.00,5129.48272,N,00002.27093,W,2,12,0.97,16.0,M,45.5,M,,0000*6D
        // $GNGGA,155339.00,5129.48271,N,00002.27103,W,2,12,0.94,15.9,M,45.5,M,,0000*6E
        // $GNGGA,155340.00,5129.48268,N,00002.27121,W,2,12,0.86,15.7,M,45.5,M,,0000*65
        #[derive(Debug)]
        pub struct GGA {
            talker_id: String,
            message_type_code: NmeaMessageTypeCode,
            utc_position_report: UTCTime, // UTC of this position report, hh is hours, mm is minutes, ss.ss is seconds.
            latitude: String, // Latitude, dd is degrees, mm.mm is minutes
            latitude_ns: String, // N or S (North or South)
            longitude: String, // Longitude, dd is degrees, mm.mm is minutes
            longitude_ew: String, // E or W (East or West)
            gps_quality_indicator: GpsQualityIndicator, // GPS Quality Indicator (non null)
            satellites_in_use: u8, // Number of satellites in use, 00 - 12
            horizontal_dilution_of_precision: f64, // Horizontal Dilution of precision (meters)
            antenna_altitude: f64, // Antenna Altitude above/below mean-sea-level (geoid) (in meters)
            antenna_altitude_units: String, // Units of antenna altitude, meters
            geoidal_separation: f64, // Geoidal separation, the difference between the WGS-84 earth ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level below ellipsoid
            geoidal_separation_units: String, // Units of geoidal separation, meters
            age_of_differential_gps_data: Option<u32>, // Age of differential GPS data, time in seconds since last SC104 type 1 or 9 update, null field when DGPS is not used
            differential_reference_station_id: Option<u16>,// Differential reference station ID, 0000-1023
            checksum: String, // Checksum
        }
        
        impl NmeaMessage for GGA {
            fn parse(message: &str) -> Self {
                // Skip leading $ and the checksum, split by commas and collect into a vector
                let parts: Vec<&str> = message[1..message.len()-3].split(',').collect();
        
                // Ensure we have enough parts
                if parts.len() != 15 {
                    log::error!("Invalid GGA message format. Expecting 15 parts, got {}.", parts.len());
                }
        
                // Extract data from parts
                let talker_id = parts[0][..2].to_string();
                let message_type_code = NmeaMessageTypeCode::GGA;
                let utc_position_report = UTCTime::new(parts[1]);
                let latitude = parts[2].to_string();
                let latitude_ns = parts[3].to_string();
                let longitude = parts[4].to_string();
                let longitude_ew = parts[5].to_string();
                let gps_quality_indicator = match parts[6].parse::<u8>() {
                    Ok(value) => GpsQualityIndicator::from(value),
                    Err(_) => GpsQualityIndicator::Invalid,
                };
                let satellites_in_use = parts[7].parse::<u8>().unwrap_or(0);
                let horizontal_dilution_of_precision = parts[8].parse::<f64>().unwrap_or(0.0);
                let antenna_altitude = parts[9].parse::<f64>().unwrap_or(0.0);
                let antenna_altitude_units = parts[10].to_string();
                let geoidal_separation = parts[11].parse::<f64>().unwrap_or(0.0);
                let geoidal_separation_units = parts[12].to_string();
                let age_of_differential_gps_data = parts[13].parse::<u32>().ok();
                let differential_reference_station_id = parts[14].parse::<u16>().ok();
        
                // Extract checksum (last two characters after asterisk)
                let checksum = message[message.len()-3..].to_string();
        
                GGA {
                    talker_id,
                    message_type_code,
                    utc_position_report,
                    latitude,
                    latitude_ns,
                    longitude,
                    longitude_ew,
                    gps_quality_indicator,
                    satellites_in_use,
                    horizontal_dilution_of_precision,
                    antenna_altitude,
                    antenna_altitude_units,
                    geoidal_separation,
                    geoidal_separation_units,
                    age_of_differential_gps_data,
                    differential_reference_station_id,
                    checksum,
                }
            }
        
            fn get_checksum(&self) -> String {
                self.checksum.clone()
            }
        
            fn get_talker_id(&self) -> String {
                self.talker_id.clone()
            }
        
            fn get_message_type_code(&self) -> &NmeaMessageTypeCode {
                &self.message_type_code
            }
        }

        // Function to parse a message and determine its type
        pub fn parse_message_type_code(message: &str) -> NmeaMessageTypeCode {
            match &message[3..6] {
                "GSA" => NmeaMessageTypeCode::GSA,
                "GGA" => NmeaMessageTypeCode::GGA,
                "GSV" => NmeaMessageTypeCode::GSV,
                // WIll add more patterns as needed
                _ => NmeaMessageTypeCode::NotImplemented,
            }
        }
        
    }
}
   