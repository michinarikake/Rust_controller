import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

def plot_log(csv_filename):
    # CSV 読み込み
    df = pd.read_csv(csv_filename, index_col=False)


    # **列名のスペースを削除**
    df.columns = df.columns.str.strip()

    # **データ型を確認**
    print(df.dtypes)

    # **各列のデータの中身をチェック**
    print(df.head())

    # **プロットディレクトリの作成**
    plot_dir = os.path.join(os.path.dirname(csv_filename), "plots")
    os.makedirs(plot_dir, exist_ok=True)

    # `mu_x` のプロット
    plt.figure(figsize=(10, 4))
    for i in range(6):
        plt.plot(df["time"], df[f"mu_x_{i}"], label=f"mu_x_{i}")
    plt.xlabel("Time Step")
    plt.ylabel("mu_x values")
    plt.title("State Variables (mu_x) Over Time")
    plt.legend()
    plt.savefig(os.path.join(plot_dir, "mu_x_plot.png"))
    plt.close()

    # `est_x` のプロット
    plt.figure(figsize=(10, 4))
    for i in range(6):
        plt.plot(df["time"], df[f"est_x_{i}"], label=f"est_x_{i}")
    plt.xlabel("Time Step")
    plt.ylabel("est_x values")
    plt.title("Estimated State Variables (est_x) Over Time")
    plt.legend()
    plt.savefig(os.path.join(plot_dir, "est_x_plot.png"))
    plt.close()

    # `P` のプロット
    plt.figure(figsize=(10, 4))
    for i in range(21):
        plt.plot(df["time"], df[f"p_{i}"], label=f"p_{i}")
        print(df[f"p_{i}"])
    plt.xlabel("Time Step")
    plt.ylabel("P values")
    plt.title("Covariance Matrix Elements (P) Over Time")
    plt.legend()
    plt.savefig(os.path.join(plot_dir, "p_plot.png"))
    plt.close()

    # f のプロット
    plt.figure(figsize=(10, 4))
    for i in range(3):
        plt.plot(df["time"], df[f"f{i}"], label=f"f_{i}")
    plt.xlabel("Time Step")
    plt.ylabel("f values")
    plt.title("Objective Function (f) Over Time")
    plt.legend()
    plt.savefig(os.path.join(plot_dir, "f_plot.png"))

if __name__ == "__main__":
    # コマンドライン引数で CSV ファイルを指定
    if len(sys.argv) > 1:
        log_filename = sys.argv[1]
    else:
        log_filename = "data/2025-02-23/2025-02-23_simulation_log.csv"  # デフォルト値

    print(f"Processing log file: {log_filename}")
    plot_log(log_filename)
