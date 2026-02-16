import rclpy
from rclpy.node import Node
from interfaces.msg import RobotState
from interfaces.srv import GetPackages
from rich.live import Live
from rich.table import Table
from rich.console import Console
from rich.panel import Panel
import time

# Konstanten für das Mapping (entsprechend deiner Definition)
STATIONS = ["A", "B", "C", "D", "E", "F", "G"]
PACKAGE_LOCATIONS = STATIONS + ["ROBOT"]
LIFECYCLE_STATES = ["SPAWNED", "READY", "WAITING", "PROCESSING", "FINISHED", "FAILED"]

class WarehouseMonitor(Node):
    def __init__(self):
        super().__init__('warehouse_monitor')
        self.robot_state = None
        self.packages = []
        self.start_time = time.monotonic()
        
        # Subscriber für Roboter-Status
        self.create_subscription(RobotState, '/robot_state', self.robot_cb, 10)
        
        # Service Client für Pakete
        self.pkg_client = self.create_client(GetPackages, '/get_packages')
        
        # Timer für die Paket-Updates (1Hz)
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
            if res is not None:
                # Sicherstellen, dass wir auf die Liste der Pakete zugreifen
                self.packages = res.packages
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def get_elapsed_time(self):
        return time.monotonic() - self.start_time

    def generate_ui(self):
        elapsed = self.get_elapsed_time()
        
        # 1. Header Bereich
        if self.robot_state:
            bat = self.robot_state.battery
            bat_col = "green" if bat > 50 else "yellow" if bat > 20 else "red"
            # Hier nutzen wir direkt den String aus dem msg oder mappen ihn, falls es ein ID ist
            pos = str(self.robot_state.robot_location)
            
            header_text = (f"[bold]Zeit seit Start:[/] {elapsed:.1f}s | "
                           f"[bold]Batterie:[/] [{bat_col}]{bat:.1f}%[/] | "
                           f"[bold]Position:[/] [cyan]{pos}[/]")
        else:
            header_text = f"[bold]Zeit seit Start:[/] {elapsed:.1f}s | [blink red]Warte auf RobotState...[/]"

        # 2. Paket Tabelle
        table = Table(show_header=True, header_style="bold magenta", expand=True)
        table.add_column("Pkg ID", justify="right", style="cyan")
        table.add_column("Location", justify="center")
        table.add_column("Lifecycle State", justify="center")
        table.add_column("Availability", justify="center")

        if not self.packages:
            table.add_row("?", "Lade Paketdaten...", "?", "?")
        else:
            # Sortierung nach ID für eine stabile Anzeige
            sorted_pkgs = sorted(self.packages, key=lambda p: p.package_idx)
            for p in sorted_pkgs:
                # --- Location Mapping ---
                # Wenn current_location ein Index ist, mappen wir ihn auf PACKAGE_LOCATIONS
                try:
                    loc_idx = int(p.current_location)
                    loc_str = PACKAGE_LOCATIONS[loc_idx] if loc_idx < len(PACKAGE_LOCATIONS) else f"ID:{loc_idx}"
                except (ValueError, TypeError):
                    # Falls es bereits ein String ist (z.B. "A")
                    loc_str = str(p.current_location)

                # --- Lifecycle Mapping ---
                try:
                    ls_idx = int(p.lifecycle_state)
                    ls_str = LIFECYCLE_STATES[ls_idx] if ls_idx < len(LIFECYCLE_STATES) else f"ID:{ls_idx}"
                except (ValueError, TypeError):
                    ls_str = str(p.lifecycle_state)
                
                # Styling für Lifecycle
                status_style = "green" if ls_str == "FINISHED" else "white"
                if ls_str == "FAILED": status_style = "bold red"
                if ls_str == "PROCESSING": status_style = "yellow"
                
                # Availability Icon
                avail = "✅ [green]YES" if p.availability else "❌ [red]NO"
                
                table.add_row(
                    str(p.package_idx),
                    loc_str,
                    f"[{status_style}]{ls_str}[/]",
                    avail
                )
        
        return Panel(table, title=header_text, title_align="left", subtitle="[italic]Warehouse Real-Time Monitor[/]")

def main():
    rclpy.init()
    monitor = WarehouseMonitor()
    console = Console()
    
    with Live(console=console, refresh_per_second=4, auto_refresh=True) as live:
        try:
            while rclpy.ok():
                # Verarbeitet ROS Callbacks
                rclpy.spin_once(monitor, timeout_sec=0.05)
                # UI Update
                live.update(monitor.generate_ui())
        except KeyboardInterrupt:
            pass
        finally:
            monitor.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()