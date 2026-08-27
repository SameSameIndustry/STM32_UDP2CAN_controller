"""
GIM6010-8 (SteadyWin, ODriveベース) の現在の設定を読むだけの、読み取り専用スクリプト。
USB経由でodrivetoolのPythonライブラリ(`odrive`パッケージ)を使って接続する。

!!! このスクリプトは SteadyWin公式の"GIM6010-8 Micromotor Instruction Manual" (rev1.2,
    https://steadywin-motor.com/products/document-download の "Document of GDS68
    Driver.zip" に含まれる) を根拠にしている。SteadyWinのGIMシリーズは ODrive公式サイト
    (docs.odriverobotics.com) の最新(v0.6系)ではなく、旧世代(legacy, 0.5系相当)の
    属性パス (axis0.motor.config.X, axis0.encoder.config.X) を使っている。
    前回作った版はv0.6系のパス(axis0.config.motor.X等)を使っており誤りだったため、
    このバージョンで実機マニュアル準拠のパスに直してある。

GIM6010-8は完成品アクチュエータなので、SteadyWin側で既に較正済みの可能性が高い。
odrive_configure.py で上書きする前に、まずこのスクリプトで「今何が設定されているか」を
確認すること。

実行方法:
    pip install odrive
    python odrive_inspect.py

参照:
  - SteadyWin GIM6010-8 Micromotor Instruction Manual rev1.2
    (Document Download: https://steadywin-motor.com/products/document-download
     -> "Document of GDS68 Driver.zip" に同梱)
"""

import odrive


def safe_get(obj, path, default="<N/A>"):
    cur = obj
    try:
        for part in path.split("."):
            cur = getattr(cur, part)
        return cur
    except AttributeError:
        return default


def main():
    print("ODriveを検索中... (USB接続を確認してください)")
    odrv0 = odrive.find_any()
    print("接続しました。\n")

    print("=== ファームウェア/ハードウェア情報 ===")
    print(f"  hw_version: {safe_get(odrv0, 'hw_version_major')}."
          f"{safe_get(odrv0, 'hw_version_minor')}."
          f"{safe_get(odrv0, 'hw_version_variant')}"
          f"  (マニュアルによるとGIM6010-8はmajor=3, minor=8)")
    print(f"  serial_number: {safe_get(odrv0, 'serial_number')}")
    print(f"  vbus_voltage: {safe_get(odrv0, 'vbus_voltage')} V")
    print(f"  ibus: {safe_get(odrv0, 'ibus')} A")

    print("\n=== Axis0 現在の状態 ===")
    print(f"  current_state: {safe_get(odrv0, 'axis0.current_state')}"
          f"  (1=IDLE, 8=CLOSED_LOOP_CONTROL)")

    print("\n=== モーター設定 (axis0.motor.config.*) ===")
    print(f"  motor_type: {safe_get(odrv0, 'axis0.motor.config.motor_type')}")
    print(f"  pole_pairs: {safe_get(odrv0, 'axis0.motor.config.pole_pairs')}"
          f"  (GIM6010-8マニュアル記載値: 14)")
    print(f"  torque_constant: {safe_get(odrv0, 'axis0.motor.config.torque_constant')}"
          f"  (GIM6010-8マニュアル記載値: 0.47 N.m/A)")
    print(f"  current_lim: {safe_get(odrv0, 'axis0.motor.config.current_lim')}"
          f"  (GIM6010-8は50Aを超えないこと、とマニュアルに明記あり)")
    print(f"  calibration_current: {safe_get(odrv0, 'axis0.motor.config.calibration_current')}"
          f"  (デフォルト5A)")
    print(f"  resistance_calib_max_voltage: "
          f"{safe_get(odrv0, 'axis0.motor.config.resistance_calib_max_voltage')}")
    print(f"  pre_calibrated: {safe_get(odrv0, 'axis0.motor.config.pre_calibrated')}")
    print(f"  phase_resistance: {safe_get(odrv0, 'axis0.motor.config.phase_resistance')}"
          f"  (マニュアル記載値: 0.48Ω±10%)")
    print(f"  phase_inductance: {safe_get(odrv0, 'axis0.motor.config.phase_inductance')}"
          f"  (マニュアル記載値: 368uH±10%)")

    print("\n=== エンコーダ設定 (axis0.encoder.config.*) ===")
    print(f"  cpr: {safe_get(odrv0, 'axis0.encoder.config.cpr')}"
          f"  (16bit絶対値エンコーダなので通常16384)")
    print(f"  pre_calibrated: {safe_get(odrv0, 'axis0.encoder.config.pre_calibrated')}")
    print(f"  index_offset (ユーザーゼロ点): {safe_get(odrv0, 'axis0.encoder.config.index_offset')}")
    print(f"  pos_estimate (現在位置, モーター軸換算[rev]): "
          f"{safe_get(odrv0, 'axis0.encoder.pos_estimate')}")
    print("  ※ マニュアルにはload_encoder/commutation_encoderのような出力軸専用エンコーダ"
          "の概念は見当たらない。pos_estimateはモーター軸(エンコーダ)基準のturnsであり、"
          "出力軸換算するにはgear_ratio(GIM6010-8は8)で割る必要がある。")

    print("\n=== コントローラ設定 (axis0.controller.config.*) ===")
    print(f"  control_mode: {safe_get(odrv0, 'axis0.controller.config.control_mode')}"
          f"  (0=Voltage,1=Torque,2=Velocity,3=Position)")
    print(f"  input_mode: {safe_get(odrv0, 'axis0.controller.config.input_mode')}"
          f"  (1=Direct,2=VelRamp,3=PosFilter,5=Trapezoidal,6=TorqueRamp,9=MIT)")
    print(f"  vel_limit: {safe_get(odrv0, 'axis0.controller.config.vel_limit')} turn/s"
          f"  (モーター軸換算。マニュアルの定格120rpm/最大420rpmは出力軸想定なら"
          f"モーター軸では ×8 = 定格16turn/s, 最大56turn/s 相当)")
    print(f"  pos_gain: {safe_get(odrv0, 'axis0.controller.config.pos_gain')}")
    print(f"  vel_gain: {safe_get(odrv0, 'axis0.controller.config.vel_gain')}")
    print(f"  vel_integrator_gain: {safe_get(odrv0, 'axis0.controller.config.vel_integrator_gain')}")

    print("\n=== CAN設定 (axis0.config.can.* / can.config.*) ===")
    print(f"  axis0.config.can.node_id: {safe_get(odrv0, 'axis0.config.can.node_id')}")
    print(f"  can.config.baud_rate: {safe_get(odrv0, 'can.config.baud_rate')}"
          f"  (デフォルト500000。STM32側は1000000前提なので合わせる必要あり)")
    print(f"  can.config.enable_r120 (終端抵抗): {safe_get(odrv0, 'can.config.enable_r120')}")
    print(f"  axis0.config.can.heartbeat_rate_ms: "
          f"{safe_get(odrv0, 'axis0.config.can.heartbeat_rate_ms')}  (デフォルト100)")
    print(f"  axis0.config.can.encoder_rate_ms: "
          f"{safe_get(odrv0, 'axis0.config.can.encoder_rate_ms')}  (デフォルト10)")

    print("\n=== 電流/電圧の限界値 (odrv0.config.*) ===")
    print(f"  dc_max_positive_current: {safe_get(odrv0, 'config.dc_max_positive_current')}")
    print(f"  dc_max_negative_current: {safe_get(odrv0, 'config.dc_max_negative_current')}")
    print(f"  dc_bus_undervoltage_trip_level: {safe_get(odrv0, 'config.dc_bus_undervoltage_trip_level')}")
    print(f"  dc_bus_overvoltage_trip_level: {safe_get(odrv0, 'config.dc_bus_overvoltage_trip_level')}")

    print("\n(値が<N/A>の場合、ファームウェアの世代がこのマニュアル想定と違う可能性がある。"
          "odrivetoolを対話的に起動して dir(odrv0.axis0) 等でツリーを見てください)")


if __name__ == "__main__":
    main()
