CREATE TABLE logs (
  uuid SERIAL PRIMARY KEY,
  timein TIMESTAMP DEFAULT now(),
  data JSON NOT NULL
);

CREATE TABLE biometrics (
  uuid SERIAL PRIMARY KEY,
  timein TIMESTAMP DEFAULT now(),
  user_id VARCHAR(20) NOT NULL,
  o2 INTEGER NOT NULL,
  battery INTEGER NOT NULL,
  bpm INTEGER NOT NULL
);

CREATE TABLE location (
  uuid SERIAL PRIMARY KEY,
  timein TIMESTAMP DEFAULT now(),
  user_id VARCHAR(20) NOT NULL,
  latitude INTEGER NOT NULL,
  longitude INTEGER NOT NULL,
  altitude INTEGER NOT NULL,
  heading INTEGER NOT NULL
);

CREATE TABLE users (
  uuid SERIAL PRIMARY KEY,
  name VARCHAR(20) NOT NULL
);

-- INSERT INTO users (userid, callsign, firstname, lastname) VALUES (101, "Maverick", "Pete", "Mitchel"), (102, "Boxman", "Jim", "Boxman"), (103, "Goose", "Nick", "Bradshaw");
-- INSERT INTO biometrics (timein, userid, o2, battery, bpm) VALUES (now(), 101, 140, 100, 100), (now(), 102, 155, 100, 100), (now(), 103, 167, 100, 100);
-- INSERT INTO location (timein, userid, latitude, longitude, altitude, heading) VALUES (now(), 101, 130, 100, 100, 100), (now(), 102, 120, 100, 100, 100), (now(), 103, 140, 100, 100, 100);