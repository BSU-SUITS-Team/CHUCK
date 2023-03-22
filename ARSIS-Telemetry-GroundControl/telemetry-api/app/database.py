import time
import psycopg2

time.sleep(10)
connection = None

try:
    connection = psycopg2.connect(dbname="groundcontrol", host="groundcontrol-db", user="SUITS", password="NASA")
    connection.autocommit = True
    print("Established connection with ground control database!")
except Exception as e:
    print("Unable to connect to database!")
    print(f"The error '{e}' occurred!")

