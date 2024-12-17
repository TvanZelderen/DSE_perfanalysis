from pathlib import Path
import pandas as pd

script_dir = Path(__file__).parent.resolve()
rasaero_path = script_dir / "rasaero" / "LAUNCH_Ring_rasaero_wo_wings.csv"
data = pd.read_csv(rasaero_path)

print(data)

