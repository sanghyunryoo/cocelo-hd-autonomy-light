#!/usr/bin/env python3
import argparse
import ipaddress
import re
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path


DEFAULT_INTERFACE = "enP8p1s0"
DEFAULT_HOST_CIDR = "192.168.1.50/24"
DEFAULT_LIDAR_IP = "192.168.1.12"
COMMON_LIDAR_IPS = ("192.168.1.12", "192.168.1.102", "192.168.1.3", "192.168.1.1")


def run(command, *, check=False, capture=True):
    kwargs = {
        "check": check,
        "text": True,
    }
    if capture:
        kwargs.update({"stdout": subprocess.PIPE, "stderr": subprocess.PIPE})
    return subprocess.run(command, **kwargs)


def default_config_path():
    return Path(__file__).resolve().parents[1] / "config" / "autonomy_light.yaml"


def parse_config(path):
    values = {
        "livox_interface": DEFAULT_INTERFACE,
        "livox_host_ip": DEFAULT_HOST_CIDR,
        "livox_lidar_ip": DEFAULT_LIDAR_IP,
    }
    if not path or not path.exists():
        return values

    text = path.read_text(encoding="utf-8")
    try:
        import yaml

        data = yaml.safe_load(text) or {}
        params = ((data.get("autonomy_light") or {}).get("ros__parameters")) or {}
        for key in values:
            if params.get(key):
                values[key] = str(params[key]).strip()
        return values
    except Exception:
        pass

    for key in values:
        match = re.search(rf"^\s*{re.escape(key)}\s*:\s*[\"']?([^\"'\n#]+)", text, re.MULTILINE)
        if match:
            values[key] = match.group(1).strip()
    return values


def ensure_cidr(value):
    return value if "/" in value else f"{value}/24"


def ip_from_cidr(cidr):
    return str(ipaddress.ip_interface(cidr).ip)


def network_from_cidr(cidr):
    return ipaddress.ip_interface(cidr).network


def interface_exists(interface):
    return Path("/sys/class/net", interface).exists()


def interface_has_cidr(interface, cidr):
    result = run(["ip", "-o", "-4", "addr", "show", "dev", interface])
    return result.returncode == 0 and cidr in result.stdout.split()


def interface_has_ip(interface, ip_addr):
    result = run(["ip", "-o", "-4", "addr", "show", "dev", interface])
    if result.returncode != 0:
        return False
    return any(part.split("/", 1)[0] == ip_addr for part in result.stdout.split())


def sudo_command(command):
    sudo = shutil.which("sudo")
    return ([sudo] if sudo else []) + command


def configure_host(interface, host_cidr):
    host_ip = ip_from_cidr(host_cidr)
    if not shutil.which("ip"):
        print("error: 'ip' command not found", file=sys.stderr)
        return False
    if not interface_exists(interface):
        print(f"error: interface does not exist: {interface}", file=sys.stderr)
        return False

    run(sudo_command(["ip", "link", "set", interface, "up"]), capture=False)
    if interface_has_cidr(interface, host_cidr) or interface_has_ip(interface, host_ip):
        return True

    print(f"configuring host network: {interface} {host_cidr}")
    result = run(sudo_command(["ip", "addr", "add", host_cidr, "dev", interface]), capture=False)
    return result.returncode == 0


def ping(ip_addr):
    return run(["ping", "-c", "1", "-W", "1", str(ip_addr)]).returncode == 0


def arping(interface, ip_addr):
    if not shutil.which("arping"):
        return False
    command = ["arping", "-c", "1", "-w", "1", "-I", interface, str(ip_addr)]
    result = run(command)
    if result.returncode == 0:
        return True
    return run(sudo_command(command)).returncode == 0


def parse_neighbors(interface, host_ip):
    result = run(["ip", "neigh", "show", "dev", interface])
    neighbors = {}
    if result.returncode != 0:
        return neighbors

    for line in result.stdout.splitlines():
        parts = line.split()
        if not parts:
            continue
        ip_addr = parts[0]
        if ip_addr == host_ip:
            continue
        state = parts[-1] if parts else ""
        if state in {"FAILED", "INCOMPLETE"}:
            continue
        mac = ""
        if "lladdr" in parts:
            index = parts.index("lladdr")
            if index + 1 < len(parts):
                mac = parts[index + 1]
        neighbors[ip_addr] = {"state": state, "mac": mac}
    return neighbors


def nmap_scan(interface, network, host_ip):
    if not shutil.which("nmap"):
        return set()
    result = run(["nmap", "-n", "-sn", "-e", interface, str(network)])
    found = set()
    if result.returncode != 0:
        return found
    for line in result.stdout.splitlines():
        marker = "Nmap scan report for "
        if marker in line:
            ip_addr = line.split(marker, 1)[1].strip().split()[-1]
            if ip_addr != host_ip:
                found.add(ip_addr)
    return found


def ordered_unique(items):
    seen = set()
    output = []
    for item in items:
        if item in seen:
            continue
        seen.add(item)
        output.append(item)
    return output


def full_ping_scan(network, host_ip, workers):
    hosts = [str(host) for host in network.hosts() if str(host) != host_ip]
    found = set()
    with ThreadPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(ping, host): host for host in hosts}
        for future in as_completed(futures):
            if future.result():
                found.add(futures[future])
    return found


def main():
    parser = argparse.ArgumentParser(
        description="Find or verify a fixed Livox MID360 IP on the autonomy-light wired network."
    )
    parser.add_argument("--config", default=str(default_config_path()), help="autonomy_light.yaml path")
    parser.add_argument("--interface", help=f"wired interface, default from config or {DEFAULT_INTERFACE}")
    parser.add_argument("--host-ip", help=f"host IP/CIDR, default from config or {DEFAULT_HOST_CIDR}")
    parser.add_argument("--lidar-ip", help=f"LiDAR IP to verify, default from config or {DEFAULT_LIDAR_IP}")
    parser.add_argument("--no-configure-host", action="store_true", help="do not add the host IP to the interface")
    parser.add_argument("--full-scan", action="store_true", help="ping-scan the full host subnet")
    parser.add_argument("--workers", type=int, default=64, help="parallel ping workers for --full-scan")
    args = parser.parse_args()

    config = parse_config(Path(args.config))
    interface = args.interface or config["livox_interface"]
    host_cidr = ensure_cidr(args.host_ip or config["livox_host_ip"])
    host_ip = ip_from_cidr(host_cidr)
    network = network_from_cidr(host_cidr)
    configured_lidar_ip = args.lidar_ip or config["livox_lidar_ip"]

    print(f"config: {args.config}")
    print(f"interface: {interface}")
    print(f"host: {host_cidr}")
    print(f"configured_lidar_ip: {configured_lidar_ip}")

    if not args.no_configure_host and not configure_host(interface, host_cidr):
        return 2

    candidates = ordered_unique([configured_lidar_ip, *COMMON_LIDAR_IPS])
    print("\nchecking fixed/common candidates:")
    reachable = set()
    for candidate in candidates:
        ping_ok = ping(candidate)
        arp_ok = arping(interface, candidate)
        if ping_ok or arp_ok:
            reachable.add(candidate)
        status = "FOUND" if candidate in reachable else "no reply"
        via = []
        if ping_ok:
            via.append("ping")
        if arp_ok:
            via.append("arp")
        suffix = f" ({', '.join(via)})" if via else ""
        print(f"  {candidate}: {status}{suffix}")

    neighbors = parse_neighbors(interface, host_ip)
    if neighbors:
        print("\nneighbor table:")
        for ip_addr, info in sorted(neighbors.items(), key=lambda item: ipaddress.ip_address(item[0])):
            mac = f" mac={info['mac']}" if info["mac"] else ""
            print(f"  {ip_addr} state={info['state']}{mac}")
            reachable.add(ip_addr)

    nmap_found = nmap_scan(interface, network, host_ip)
    if nmap_found:
        print("\nnmap discovered:")
        for ip_addr in sorted(nmap_found, key=ipaddress.ip_address):
            print(f"  {ip_addr}")
        reachable.update(nmap_found)

    if args.full_scan:
        print(f"\nfull ping scan: {network}")
        scanned = full_ping_scan(network, host_ip, max(1, args.workers))
        for ip_addr in sorted(scanned, key=ipaddress.ip_address):
            print(f"  {ip_addr}")
        reachable.update(scanned)

    reachable.discard(host_ip)
    print("\nresult:")
    if configured_lidar_ip in reachable:
        print(f"  OK: configured livox_lidar_ip responds: {configured_lidar_ip}")
        return 0

    if len(reachable) == 1:
        found = next(iter(reachable))
        print(f"  likely MID360 IP: {found}")
        print("  set config/autonomy_light.yaml livox_lidar_ip to this value.")
        return 1

    if reachable:
        print("  multiple devices found; choose the MID360 IP from this list:")
        for ip_addr in sorted(reachable, key=ipaddress.ip_address):
            print(f"    {ip_addr}")
        return 1

    print("  no device found on the configured subnet.")
    print("  check power/cable, then try --full-scan or Livox Viewer.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
