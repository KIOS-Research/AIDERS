import os
import sys
import time
import threading
from concurrent.futures import ThreadPoolExecutor
from kafka import KafkaAdminClient
from dotenv import load_dotenv

import database.connection
import subscribers
import api

# Interval (in seconds) between Kafka health checks
INTERVAL = 5

#wrapper function to supervise kafka connection
def supervised_consume(create_consumer_fn, consume_fn, *args):
    backoff = 1
    while True:
        consumer = None
        try:
            consumer = create_consumer_fn()
            consume_fn(consumer, *args)
            # If your consume_fn ever returns normally raise disconnection error
            raise RuntimeError("consumer is disconnected unexpectedly")
        except Exception as e:
            print(f"[supervisor] Unexpected error, retrying in {backoff}s: {e}")
        finally:
            if consumer:
                try: 
                    consumer.close() 
                except: 
                    pass

        time.sleep(backoff)
        # backoff = min(backoff * 2, 30)  # exponential back‐off to 30s
        backoff += 1

def main():

    # load environment variables from .env
    load_dotenv()

    if os.environ.get("KAFKA_ACTIVE") != "1":
        print("🔕 Kafka is not active. Exiting...")
        sys.exit(0)

    # initialize database connection pool
    poolCount = int(os.getenv('KC_DB_CONNECTION_POOLS', 5))
    database.connection.init(poolCount, "kcPool")


    # start consumers (with reconnect wrapper)
    # executor = ThreadPoolExecutor(max_workers=3)
    # executor.submit(
    #     supervised_consume,
    #     subscribers.createObjectDetectionConsumer,
    #     subscribers.consumeObjectDetection
    # )
  
    # executor.submit(
    #     supervised_consume,
    #     subscribers.createCrisisClassificationConsumer,
    #     subscribers.consumeCrisisClassification
    # )

    # executor.submit(
    #     supervised_consume,
    #     subscribers.createPathPlanningOutputConsumer,
    #     subscribers.consumePathPlanningOutput
    # )


    # start consumers (no reconnect wrapper)
    executor = ThreadPoolExecutor(max_workers=3)
    executor.submit(
        subscribers.consumeObjectDetection,
        subscribers.createObjectDetectionConsumer()
    )
    executor.submit(
        subscribers.consumeCrisisClassification,
        subscribers.createCrisisClassificationConsumer()
    )
    executor.submit(
        subscribers.consumePathPlanningOutput,
        subscribers.createPathPlanningOutputConsumer()
    )


    # start the HTTP API (this call blocks)
    apiPort = int(os.getenv('KC_API_PORT'))
    api.start(apiPort)

if __name__ == '__main__':
    main()


# def loop(brokerUrl, interval=INTERVAL):
#     # background thread: repeatedly health-check Kafka
#     while True:
#         try:
#             admin = KafkaAdminClient(bootstrap_servers=brokerUrl)
#             admin.list_topics()
#             admin.close()
#             print(f"[health] Connected to Kafka at {brokerUrl}")
#         except Exception as e:
#             print(f"[health] Kafka connection error: {e}")
#         time.sleep(interval)

# def checkKafkaConnectionStatus(brokerUrl, interval=INTERVAL):
#     # start the Kafka health-check thread as a daemon
#     thread = threading.Thread(target=loop, args=(brokerUrl, interval), daemon=True)
#     thread.start()