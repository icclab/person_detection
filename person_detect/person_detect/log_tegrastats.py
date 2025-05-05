import subprocess
import time

class TegrastatsLogger:
    def __init__(self):

        # Start tegrastats
        interval_ms = 100
        self.proc = subprocess.Popen(["tegrastats", "--interval", str(interval_ms)], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

        self.prev_instantaneous_mW = 0
        self.prev_average_mW = 0
        self.prev_unix_time = 0
        self.energy_J = 0
        self.energy_total_J = 0

        self.i = True

    def log_tegrastats(self):

        line = self.proc.stdout.readline()

        if not line:
            print("[TegrastatsLogger] No output from tegrastats.")
            return 0, 0, 0, 0, 0
        
        if self.i:
            prev_line = line
            prev_parts = prev_line.strip().split()

            self.prev_unix_time = time.time()

            print(f"unix_time: {self.prev_unix_time}")

            prev_vdd_in_index = prev_parts.index("VDD_IN")
            prev_vdd = prev_parts[prev_vdd_in_index+1]
            
            prev_instantaneous_str, prev_average_str = prev_vdd.split("/")        
            self.prev_instantaneous_mW = float(prev_instantaneous_str.replace("mW", ""))
            self.prev_average_mW = float(prev_average_str.replace("mW", ""))

            self.i = False

            return self.prev_unix_time, self.prev_instantaneous_mW, self.prev_average_mW, self.energy_J, self.energy_total_J

        # Split the line into parts
        parts = line.strip().split()

        # print(parts)

        unix_time = time.time()

        vdd_in_index = parts.index("VDD_IN")
        vdd = parts[vdd_in_index+1]

        instantaneous_str, average_str = vdd.split("/")
        
        instantaneous_mW = float(instantaneous_str.replace("mW", ""))
        average_mW = float(average_str.replace("mW", ""))

        avg_power_mW = (self.prev_instantaneous_mW + instantaneous_mW) / 2
        elapsed_sec = unix_time - self.prev_unix_time

        self.energy_J = (avg_power_mW * elapsed_sec) / 1000
        self.energy_total_J += self.energy_J

        print(f"unix_time: {unix_time}")
        print(f"Instantaneous VDD_IN: {instantaneous_mW} mW")
        print("prev_instantaneous_mW:", self.prev_instantaneous_mW)
        # print(f"Average VDD_IN: {average_mW} mW")
        print("average_power_mW: ", avg_power_mW)
        print("elapsed_sec: ", elapsed_sec)
        print("energy_J: ", self.energy_J)

        self.prev_instantaneous_mW = instantaneous_mW
        self.prev_unix_time = unix_time

        return unix_time, instantaneous_mW, average_mW, self.energy_J, self.energy_total_J

    def close(self):
        if self.proc:
            self.proc.terminate()
        print("[TegrastatsLogger] Logging stopped and resources cleaned up.")

if __name__ == "__main__":
    logger = TegrastatsLogger()
