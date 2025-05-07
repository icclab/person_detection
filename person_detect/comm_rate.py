import psutil
import time
import csv
import os
from datetime import datetime

# CONFIG
interface = 'wlP1p1s0'
interval = 1  # seconds
# csv_file = f'network_rate_{interface}.csv'

# Ensure interface exists
if interface not in psutil.net_io_counters(pernic=True):
    raise ValueError(f"Interface '{interface}' not found. Available: {list(psutil.net_io_counters(pernic=True).keys())}")

# # Initialize CSV
# write_header = not os.path.exists(csv_file)
# with open(csv_file, 'a', newline='') as f:
#     writer = csv.writer(f)
#     if write_header:
#         writer.writerow(['Timestamp', 'Bytes_Sent_per_s', 'Bytes_Recv_per_s'])

# Monitor and log
def monitor():
    old = psutil.net_io_counters(pernic=True)[interface]
    old_sent = old.bytes_sent
    old_recv = old.bytes_recv
    while True:
        time.sleep(interval)
        new = psutil.net_io_counters(pernic=True)[interface]
        new_sent = new.bytes_sent
        new_recv = new.bytes_recv
        sent_rate = (new_sent - old_sent) / interval
        recv_rate = (new_recv - old_recv) / interval
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        with open(csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, f"{sent_rate:.2f}", f"{recv_rate:.2f}"])
        old_sent, old_recv = new_sent, new_recv

if __name__ == "__main__":
    monitor()
