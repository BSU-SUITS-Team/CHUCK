FROM postgres:latest

ENV POSTGRES_USER=SUITS \
  POSTGRES_PASSWORD=NASA \
  POSTGRES_DB=groundcontrol

COPY ./database.sql /docker-entrypoint-initdb.d/