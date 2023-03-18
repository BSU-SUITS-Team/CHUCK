CREATE TABLE IF NOT EXISTS logs (
  uuid SERIAL PRIMARY KEY NOT NULL,
  createdAt TIMESTAMP DEFAULT now(),
  data JSON NOT NULL
);

CREATE TABLE IF NOT EXISTS users (
  id SERIAL PRIMARY KEY NOT NULL,
  name VARCHAR(255) NOT NULL,
  createdAt TIMESTAMP DEFAULT now()
);

CREATE TABLE IF NOT EXISTS locations (
  id SERIAL PRIMARY KEY REFERENCES users,
  longitude INTEGER NOT NULL,
  latitude INTEGER NOT NULL,
  altitude INTEGER NOT NULL,
  heading INTEGER NOT NULL,
  createdAt TIMESTAMP DEFAULT now(),
  updatedAt TIMESTAMP DEFAULT now()
);

CREATE TABLE IF NOT EXISTS biometrics (
  id SERIAL PRIMARY KEY REFERENCES users,
  o2 INTEGER NOT NULL,
  battery INTEGER NOT NULL,
  heartrate INTEGER NOT NULL,
  createdAt TIMESTAMP DEFAULT now(),
  updatedAt TIMESTAMP DEFAULT now()
)

INSERT INTO logs (timeIn, data) VALUES (now(), '{"test log": "This is a test log"}'), (now(), '{"test log": "This is another test log"}');
