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
  longitude INTEGER NOT NULL,
  latitude INTEGER NOT NULL,
  altitude INTEGER NOT NULL,
  heading INTEGER NOT NULL,
  createdAt TIMESTAMP DEFAULT now(),
  updatedAt TIMESTAMP DEFAULT now()
);

CREATE TABLE IF NOT EXISTS biometrics (
  uuid uuid DEFAULT gen_random_uuid() PRIMARY KEY,
  id SERIAL REFERENCES users,
  o2 INTEGER NOT NULL,
  battery INTEGER NOT NULL,
  heartrate INTEGER NOT NULL,
  createdAt TIMESTAMP DEFAULT now(),
  updatedAt TIMESTAMP DEFAULT now()
);

INSERT INTO logs (createdAt, data) VALUES (now(), '{"test log": "This is a test log"}'), (now(), '{"test log": "This is another test log"}');
