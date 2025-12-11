#!/bin/bash

# 作業ディレクトリに移動（超重要：parent/childを呼び出すため）
cd /home/ras4/ble_test || exit 1

echo "=== System Loop Started ==="

while true; do
    # 1. 前回のゴミが残っているかもしれないので強制停止・掃除
    echo ">>> Cleaning up (stop_dual)..."
    ./stop_dual

    # 少し待機
    sleep 2

    # 2. ボタン待機プログラムを実行
    # button_dualは、内部でparent/childを実行し、
    # マッピング終了後に終了コードを返して終了します。
    echo ">>> Starting Button/Standby (button_dual)..."
    ./button_dual

    # button_dualが終了したということは、マッピングが終わった（またはエラー）ということ
    echo ">>> Cycle finished. Restarting loop in 3 seconds..."

    # 3. 再ループ前の待機（連打防止）
    sleep 3
done
