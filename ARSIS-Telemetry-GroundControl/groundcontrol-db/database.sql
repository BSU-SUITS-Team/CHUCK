CREATE TABLE logs (
  uuid SERIAL PRIMARY KEY NOT NULL,
  timeIn TIMESTAMP NOT NULL DEFAULT now(),
  log JSON NOT NULL
);

INSERT INTO logs (timeIn, log) VALUES (now(), '{"test log": "This is a test log"}'), (now(), '{"test log": "This is another test log"}');