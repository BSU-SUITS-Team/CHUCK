CREATE TABLE IF NOT EXISTS logs (
  uuid SERIAL PRIMARY KEY,
  data JSON NOT NULL,
  createdAt TIMESTAMP DEFAULT now()
);

CREATE TABLE IF NOT EXISTS users (
  id SERIAL PRIMARY KEY,
  name VARCHAR(255) UNIQUE,
  createdAt TIMESTAMP DEFAULT now()
);

CREATE TABLE IF NOT EXISTS dcu (
  uuid uuid DEFAULT gen_random_uuid() PRIMARY KEY,
  eva INTEGER NOT NULL, -- differentiate between eva1 and eva2
  batt BOOLEAN NOT NULL,
  oxy BOOLEAN NOT NULL,
  COMM BOOLEAN NOT NULL,
  FAN BOOLEAN NOT NULL,
  PUMP BOOLEAN NOT NULL,
  CO2 BOOLEAN NOT NULL,
  createdAt TIMESTAMP DEFAULT now(),
  updatedAt TIMESTAMP DEFAULT now()
)

CREATE TABLE IF NOT EXISTS imu (
  uuid uuid DEFAULT gen_random_uuid() PRIMARY KEY,
  eva INTEGER NOT NULL, -- differentiate between eva1 and eva2
  posx DECIMAL NOT NULL,
  posy DECIMAL NOT NULL,
  heading DECIMAL NOT NULL,
  createdAt TIMESTAMP DEFAULT now(),
  updatedAt TIMESTAMP DEFAULT now()
)

CREATE TABLE IF NOT EXISTS rover (
  uuid uuid DEFAULT gen_random_uuid() PRIMARY KEY,
  posx DECIMAL NOT NULL,
  posy DECIMAL NOT NULL,
  qr_id INTEGER NOT NULL,
  createdAt TIMESTAMP DEFAULT now(),
  updatedAt TIMESTAMP DEFAULT now()
)

CREATE TABLE IF NOT EXISTS locations (
  uuid uuid DEFAULT gen_random_uuid() PRIMARY KEY,
  id SERIAL REFERENCES users,
  longitude DECIMAL NOT NULL,
  latitude DECIMAL NOT NULL,
  altitude DECIMAL NOT NULL,
  heading DECIMAL NOT NULL,
  createdAt TIMESTAMP DEFAULT now(),
  updatedAt TIMESTAMP DEFAULT now()
);

CREATE TABLE IF NOT EXISTS biometrics (
  uuid uuid DEFAULT gen_random_uuid() PRIMARY KEY,
  id SERIAL REFERENCES users,
  heartrate INTEGER NOT NULL,
  o2 INTEGER NOT NULL,
  battery INTEGER NOT NULL,
  fan INTEGER NOT NULL,
  vent BOOLEAN NOT NULL,
  co2 INTEGER NOT NULL,
  sop BOOLEAN NOT NULL,
  suitPressure INTEGER NOT NULL,
  createdAt TIMESTAMP DEFAULT now(),
  updatedAt TIMESTAMP DEFAULT now()
);

CREATE TABLE IF NOT EXISTS uia (
  uuid uuid DEFAULT gen_random_uuid() PRIMARY KEY,
  eva1_power BOOLEAN NOT NULL,
  eva1_oxy  BOOLEAN NOT NULL,
  eva1_water_supply BOOLEAN NOT NULL,
  eva1_water_waste BOOLEAN NOT NULL,
  eva2_power BOOLEAN NOT NULL,
  eva2_oxy  BOOLEAN NOT NULL,
  eva2_water_supply BOOLEAN NOT NULL,
  eva2_water_waste BOOLEAN NOT NULL,
  oxy_vent BOOLEAN NOT NULL,
  depress BOOLEAN NOT NULL,
  createdAt TIMESTAMP DEFAULT now(),
  updatedAt TIMESTAMP DEFAULT now()
);

CREATE TABLE IF NOT EXISTS telemetry (
  uuid uuid DEFAULT gen_random_uuid() PRIMARY KEY,
  eva_time INTEGER NOT NULL,
  eva1_batt_time_left DECIMAL NOT NULL,
  eva1_oxy_pri_storage DECIMAL NOT NULL,
  eva1_oxy_sec_storage DECIMAL NOT NULL,
  eva1_oxy_pri_pressure DECIMAL NOT NULL,
  eva1_oxy_sec_pressure DECIMAL NOT NULL,
  eva1_oxy_time_left DECIMAL NOT NULL,
  eva1_heart_rate DECIMAL NOT NULL,
  eva1_oxy_consumption DECIMAL NOT NULL,
  eva1_co2_production DECIMAL NOT NULL,
  eva1_suit_pressure_oxy DECIMAL NOT NULL,
  eva1_suit_pressure_co2 DECIMAL NOT NULL,
  eva1_suit_pressure_other DECIMAL NOT NULL,
  eva1_suit_pressure_total DECIMAL NOT NULL,
  eva1_fan_pri_rpm DECIMAL NOT NULL,
  eva1_fan_sec_rpm DECIMAL NOT NULL,
  eva1_helmet_pressure_co2 DECIMAL NOT NULL,
  eva1_scrubber_a_co2_storage DECIMAL NOT NULL,
  eva1_scrubber_b_co2_storage DECIMAL NOT NULL,
  eva1_temperature DECIMAL NOT NULL,
  eva1_coolant_ml DECIMAL NOT NULL,
  eva1_coolant_gas_pressure DECIMAL NOT NULL,
  eva1_coolant_liquid_pressure DECIMAL NOT NULL,
  eva2_batt_time_left DECIMAL NOT NULL,
  eva2_oxy_pri_storage DECIMAL NOT NULL,
  eva2_oxy_sec_storage DECIMAL NOT NULL,
  eva2_oxy_pri_pressure DECIMAL NOT NULL,
  eva2_oxy_sec_pressure DECIMAL NOT NULL,
  eva2_oxy_time_left DECIMAL NOT NULL,
  eva2_heart_rate DECIMAL NOT NULL,
  eva2_oxy_consumption DECIMAL NOT NULL,
  eva2_co2_production DECIMAL NOT NULL,
  eva2_suit_pressure_oxy DECIMAL NOT NULL,
  eva2_suit_pressure_co2 DECIMAL NOT NULL,
  eva2_suit_pressure_other DECIMAL NOT NULL,
  eva2_suit_pressure_total DECIMAL NOT NULL,
  eva2_fan_pri_rpm DECIMAL NOT NULL,
  eva2_fan_sec_rpm DECIMAL NOT NULL,
  eva2_helmet_pressure_co2 DECIMAL NOT NULL,
  eva2_scrubber_a_co2_storage DECIMAL NOT NULL,
  eva2_scrubber_b_co2_storage DECIMAL NOT NULL,
  eva2_temperature DECIMAL NOT NULL,
  eva2_coolant_ml DECIMAL NOT NULL,
  eva2_coolant_gas_pressure DECIMAL NOT NULL,
  eva2_coolant_liquid_pressure DECIMAL NOT NULL,
  createdAt TIMESTAMP DEFAULT now(),
  updatedAt TIMESTAMP DEFAULT now()
)

CREATE TABLE IF NOT EXISTS eva (
  uuid uuid DEFAULT gen_random_uuid() PRIMARY KEY,
  started BOOLEAN NOT NULL,
  paused BOOLEAN NOT NULL,
  completed BOOLEAN NOT NULL,
  total_time INTEGER NOT NULL,
  uia_started BOOLEAN NOT NULL,
  uia_completed BOOLEAN NOT NULL,
  uia_time INTEGER NOT NULL,
  dcu_started BOOLEAN NOT NULL,
  dcu_completed BOOLEAN NOT NULL,
  dcu_time INTEGER NOT NULL,
  rover_started BOOLEAN NOT NULL,
  rover_completed BOOLEAN NOT NULL,
  rover_time INTEGER NOT NULL,
  spec_started BOOLEAN NOT NULL,
  spec_completed BOOLEAN NOT NULL,
  spec_time INTEGER NOT NULL,
  createdAt TIMESTAMP DEFAULT now(),
  updatedAt TIMESTAMP DEFAULT now()
)

-- INSERT INTO uia (panel_id, o2, power_, comm) VALUES (1, TRUE, TRUE, TRUE);
INSERT INTO logs (createdAt, data) VALUES (now(), '{"test log": "This is a test log"}'), (now(), '{"test log": "This is another test log"}');
