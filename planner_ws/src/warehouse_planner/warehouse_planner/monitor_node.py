import rclpy
from rclpy.node import Node
from interfaces.msg import RobotState
from interfaces.srv import GetPackages
from rich.live import Live
from rich.table import Table
from rich.console import Console
from rich.panel import Panel
import time

LIFECYCLE_STATES = ["SPAWNED", "READY", "WAITING", "PROCESSING", "FINISHED", "FAILED"]
LOC_MAP = {0: "A", 1: "B", 2: "C", 3: "D", 4: "E", 5: "F", 6: "G", 7: "H"}

class WarehouseMonitor(Node):
    def __init__(self):
        super().__init__('warehouse_monitor')
        self.robot_state = None
        self.packages = []
        self.start_time = time.monotonic()
        
        self.create_subscription(RobotState, '/robot_state', self.robot_cb, 10)
        
        self.pkg_client = self.create_client(GetPackages, '/get_packages')
        
        self.create_timer(1.0, self.timer_fetch_packages)

    def robot_cb(self, msg):
        self.robot_state = msg

    def timer_fetch_packages(self):
        if not self.pkg_client.service_is_ready():
            return
        req = GetPackages.Request()
        req.all = True
        future = self.pkg_client.call_async(req)
        future.add_done_callback(self.package_callback)

    def package_callback(self, future):
        try:
            res = future.result()
            if res and hasattr(res, 'packages'):
                self.packages = res.packages
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def generate_ui(self):
        elapsed = time.monotonic() - self.start_time
        
        if self.robot_state:
            bat = self.robot_state.battery
            bat_col = "green" if bat > 50 else "yellow" if bat > 20 else "red"
            
            # Carrying Info
            c_idx = getattr(self.robot_state, 'carrying_idx', -1)
            carrying_str = f"Paket {c_idx}" if c_idx != -1 else "Leer"
            
            # Action Info 
            last_action = getattr(self.robot_state, 'last_action', "Warte...")

            header_text = (f"[bold]Zeit:[/] {elapsed:.1f}s | "
                           f"[bold]Batterie:[/] [{bat_col}]{bat:.1f}%[/] | "
                           f"[bold]Pos:[/] {self.robot_state.robot_location} | "
                           f"[bold]Carrying:[/] [cyan]{carrying_str}[/] | "
                           f"[bold]Action:[/] [magenta]{last_action}[/]")
        else:
            header_text = f"[bold]Zeit:[/] {elapsed:.1f}s | Warte auf RobotState..."

        # --- Table ---
        table = Table(show_header=True, header_style="bold magenta", expand=True)
        table.add_column("Pkg ID", justify="right", style="cyan", width=8)
        table.add_column("Location", justify="center")
        table.add_column("Lifecycle State", justify="center")
        table.add_column("Availability", justify="center", width=12)

        if not self.packages:
            table.add_row("-", "Lade Paketdaten...", "-", "-")
        else:
            sorted_pkgs = sorted(self.packages, key=lambda p: p.package_idx)
            for p in sorted_pkgs:
                # Lifecycle
                raw_state = p.lifecycle_state
                try:
                    ls_idx = int(raw_state)
                    ls_str = LIFECYCLE_STATES[ls_idx] if ls_idx < len(LIFECYCLE_STATES) else str(ls_idx)
                except ValueError:
                    ls_str = str(raw_state)
                
                # Location Mapping
                raw_loc = p.current_location
                try:
                    loc_idx = int(raw_loc)
                    loc_str = LOC_MAP.get(loc_idx, f"Station {loc_idx}")
                except ValueError:
                    loc_str = str(raw_loc)
                
                # Style
                status_style = "green" if ls_str == "FINISHED" else "white"
                if ls_str == "FAILED": status_style = "bold red"
                if ls_str == "PROCESSING": status_style = "blue"
                
                table.add_row(
                    str(p.package_idx),
                    loc_str,
                    f"[{status_style}]{ls_str}[/]",
                    "✅" if p.availability else "❌"
                )
        
        return Panel(table, title=header_text, subtitle="Warehouse Real-Time Monitor", border_style="blue")

def main():
    rclpy.init()
    monitor = WarehouseMonitor()
    console = Console()
    
    with Live(console=console, refresh_per_second=4) as live:
        try:
            while rclpy.ok():
                rclpy.spin_once(monitor, timeout_sec=0.1)
                live.update(monitor.generate_ui())
        except KeyboardInterrupt:
            pass

    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()