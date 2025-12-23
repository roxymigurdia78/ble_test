#!/bin/bash
# 作業ディレクトリに移動
cd /home/kaiji/ble_test || exit 1

echo "=== System Loop Started ==="

while true; do
    # ------------------------------------------------
    # 1. リセット & クリーンアップ
    # ------------------------------------------------
    echo ">>> Cleaning up..."
    ./stop_dual
    sleep 2

    # ------------------------------------------------
    # 2. 待機 & モード選択 (button_dual)
    # ------------------------------------------------
    # 青LED点灯状態でボタン入力を待ちます
    echo ">>> Starting Button Wait..."
    ./button_dual

    # 終了コードを取得
    EXIT_CODE=$?

    # ------------------------------------------------
    # 3. 分岐処理
    # ------------------------------------------------

    # パターンA: ソロモード (コード 99)
    if [ $EXIT_CODE -eq 99 ]; then
        echo ">>> SOLO MODE Selected!"
        echo ">>> Starting effect_solo..."

        # effect_solo を実行
        # (ボタンを押すと終了してループ先頭へ戻る)
        ./effect_solo

        echo ">>> Solo mode finished. Restarting cycle..."

    # パターンB: ネットワークモード成功 (コード 0)
    elif [ $EXIT_CODE -eq 0 ]; then
        echo ">>> Network Mapping Success!"
        echo ">>> Starting effect_main..."

        # effect_main を実行
        # (ボタンを押すと終了してループ先頭へ戻る)
        ./effect_main

        echo ">>> Network effect finished. Restarting cycle..."

    # パターンC: 失敗またはその他
    else
        echo ">>> Mapping Failed or Cancelled (Code: $EXIT_CODE)."
        echo ">>> Retrying..."
    fi

    echo ">>> Restarting cycle in 3 seconds..."
    sleep 3
done
