CREATE TABLE IF NOT EXISTS logs (
  uuid SERIAL PRIMARY KEY NOT NULL,
  createdAt TIMESTAMP DEFAULT now(),
  data JSON NOT NULL
);

CREATE TABLE IF NOT EXISTS users (
  id SERIAL PRIMARY KEY,
  name VARCHAR(255) UNIQUE,
  createdAt TIMESTAMP DEFAULT now()
);

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
  createdAt TIMESTAMP DEFAULT now(),
  updatedAt TIMESTAMP DEFAULT now()
);

CREATE TABLE IF NOT EXISTS uia (
  uuid uuid DEFAULT gen_random_uuid() PRIMARY KEY,
  panel_id INTEGER NOT NULL,
  o2 BOOLEAN NOT NULL,
  power_ BOOLEAN NOT NULL,
  comm BOOLEAN NOT NULL,
  createdAt TIMESTAMP DEFAULT now(),
  updatedAt TIMESTAMP DEFAULT now()
);

INSERT INTO uia (panel_id, o2, power_, comm) VALUES (1, TRUE, TRUE, TRUE);
INSERT INTO logs (createdAt, data) VALUES (now(), '{"test log": "This is a test log"}'), (now(), '{"test log": "This is another test log"}');
