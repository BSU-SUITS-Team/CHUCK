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

-- INSERT INTO uia (panel_id, o2, power_, comm) VALUES (1, TRUE, TRUE, TRUE);
INSERT INTO logs (createdAt, data) VALUES (now(), '{"test log": "This is a test log"}'), (now(), '{"test log": "This is another test log"}');
