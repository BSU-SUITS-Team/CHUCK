CREATE TABLE logs (
  uuid SERIAL PRIMARY KEY NOT NULL,
  timein TIMESTAMP DEFAULT now(),
  data JSON NOT NULL
);

INSERT INTO logs (timeIn, data) VALUES (now(), '{"test log": "This is a test log"}'), (now(), '{"test log": "This is another test log"}');