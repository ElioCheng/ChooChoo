# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "rich",
# ]
# ///
from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from rich.text import Text


regions = []

def add_region(start, name, size=0, end=None):
    if end is None:
        end = start + size
    regions.append((start, end, name))

# Start address, End address, name
# add_region(0x00000000, "SDRAM (ARM)", 0x40000000)
# add_region(0xFE000000, "Peripherals", 0x00100000)
# add_region(0xFE201000, "UART", 0x00001000)
# add_region(0xFE200000, "GPIO", 0x00001000)

add_region(0x00000000, "REGION 1", end=0xFE000000)
add_region(0xFE000000, "REGION 2", end=0xFFFFFFFF)
# Sort regions by start address
regions.sort(key=lambda x: x[0])

# Print memory map visualization
def format_size(size):
    if size >= 1024*1024*1024:
        return f"{size/1024/1024/1024:.1f}GB"
    if size >= 1024*1024:
        return f"{size/1024/1024:.1f}MB"
    if size >= 1024:
        return f"{size/1024:.1f}KB"
    return f"{size}B"

console = Console()
# Create table for memory map
table = Table(title="Memory Map Visualization", show_header=False, box=None, padding=1)
table.add_column("Address", style="cyan")
table.add_column("Region", style="bright_white")
table.add_column("Size", style="green")


# Calculate the maximum name length for alignment
max_name_length = max(len(name) for _, _, name in regions)
box_width = max_name_length + 4  # Add padding for box characters

prev_end = 0
for start, end, name in regions:
    # Add gap row if there is one
    if start > prev_end and prev_end != 0:
        gap_size = start - prev_end
        table.add_row(
            f"0x{prev_end:08x}",
            Text("┄" * box_width + " unused", style="dim"),
            Text(f"({format_size(gap_size)})", style="dim")
        )
        table.add_row("", "", "") # Add empty row for spacing

    size = end - start
    # Add region start
    table.add_row(
        f"0x{start:08x}",
        f"┌{'─' * box_width}┐",
        ""
    )
    # Center the name within the box
    padding = (box_width - len(name)) // 2
    table.add_row(
        "",
        f"│{' ' * padding}{name}{' ' * (box_width - len(name) - padding)}│",
        ""
    )
    # Add region end
    table.add_row(
        f"0x{end:08x}",
        f"└{'─' * box_width}┘",
        f"({format_size(size)})"
    )
    table.add_row("", "", "") # Add empty row for spacing

    prev_end = end

# Display the table in a panel
console.print(Panel(table, expand=False))
