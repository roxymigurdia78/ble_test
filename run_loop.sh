#!/bin/bash
cd /home/ras4/ble_test || exit 1

while true; do
    # 1. 前回の状態をクリーンアップ
    sudo ./stop_dual
    sleep 2

    # 2. ボタン待機 & マッピング (青LED点灯)
    # 成功すると Exit Code 0 が返る
    sudo ./button_dual
    EXIT_CODE=$?

    if [ $EXIT_CODE -eq 0 ]; then
        # 3. 成功時: エフェクトモード (ボタンで終了)
        sudo ./effect_main
    else
        # 失敗時: 少し待って最初に戻る (赤LED点滅済み)
        sleep 3
    fi

    sleep 1
done
