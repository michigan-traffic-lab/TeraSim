from pathlib import Path
import typer
import json

app = typer.Typer()


@app.command()
def merge_json(dir: str, name: str, nth: str):
    dir_path = Path(dir) / name / "raw_data" / nth
    monitor_json_path = dir_path / "monitor.json"

    merged_dir_path = Path(dir) / name / "aggregated_data"
    merged_dir_path.mkdir(parents=True, exist_ok=True)

    merged_json_path = merged_dir_path / f"monitor_{nth}.json"

    # if merged json not exist, then create it with monitor json, else, read and update it with the key as nth and the value as monitor json
    if not merged_json_path.exists():
        with open(monitor_json_path) as f:
            monitor_json = json.load(f)
        with open(merged_json_path, "w") as f:
            json.dump({nth: monitor_json}, f)
        typer.echo(f"Merged JSON data written to {merged_json_path}")
    else:
        with open(merged_json_path) as f:
            merged_json = json.load(f)
        with open(monitor_json_path) as f:
            monitor_json = json.load(f)
        merged_json[nth] = monitor_json
        with open(merged_json_path, "w") as f:
            json.dump(merged_json, f)
        typer.echo(f"Merged JSON data written to {merged_json_path}")


if __name__ == "__main__":
    app()
