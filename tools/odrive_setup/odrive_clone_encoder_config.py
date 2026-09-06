"""
「動いている GIM6010-8」の ODrive 設定を JSON にダンプし、その中のエンコーダ関連
設定だけを「GIM8108-8(node 4/5)」へ書き込むためのスクリプト。

---- なぜ clone 方式なのか ----
SteadyWin GIM シリーズのデュアルエンコーダ(出力軸側エンコーダを位置フィードバックに
使う設定)を有効化する odrivetool のパラメータ名・値は、機種/ファーム個別で、公開
マニュアル(GIM6010-8 rev1.2、manuals.plus の GIM8108-8 ユーザーマニュアル)には
記載がありません。値をこちらで推測して書き込むと較正に失敗したりモーターが暴走する
恐れがあるため、**「すでに正しく設定されている個体(動いている GIM6010-8)から
読み取って、同じ値を書き戻す」** 方式にしています。ユーザーの申告どおり
GIM6010 と GIM8108 のファーム世代が同じであることが前提です。

per-unit(個体固有)の較正結果 —— 位相オフセット・エンコーダ方向・phase_resistance
など —— は clone しません(除外正規表現で弾く)。それらは書き込み後の
odrive_configure.py の較正で作り直します。

---- 使い方 ----
1. 動いている GIM6010-8 を USB 接続し、設定をダンプ:
       python odrive_clone_encoder_config.py --dump gim6010_ref.json --can-id 6

2. 中身を確認(エンコーダ関連だけ抜粋表示):
       python odrive_clone_encoder_config.py --show gim6010_ref.json

2b. 設定の差分を取る(--diff):
       # JSON 2つを比較 (USB不要)
       python odrive_clone_encoder_config.py --diff good.json bad.json
       # 保存済み JSON と「いま USB 接続中の個体」を比較
       python odrive_clone_encoder_config.py --diff good.json device --can-id 4
       # 較正前後の自分自身を比較
       python odrive_clone_encoder_config.py --dump before.json --can-id 4
       # ...較正...
       python odrive_clone_encoder_config.py --diff before.json device --can-id 4
       # .config. だけでなく全項目(毎回変わるライブ値は自動除外):
       python odrive_clone_encoder_config.py --diff a.json b.json --all

3. GIM8108-8 を USB 接続し、エンコーダ関連設定を「ドライラン」で差分確認:
       python odrive_clone_encoder_config.py --apply gim6010_ref.json --can-id 4

4. 問題なければ実際に書き込み + 保存:
       python odrive_clone_encoder_config.py --apply gim6010_ref.json --can-id 4 --write --save

   その後 odrive_configure.py で較正(--skip-calibration を付けない)して個体固有値を
   作り直し、diagnose_gim_axes.py / odrive_position_test.py で出力軸の絶対角が
   電源サイクルをまたいで再現するか確認する。

⚠ USB接続は odrive.find_any() で「今つながっている1台」を掴む。--can-id は接続直後に
   axis0.config.can.node_id と突き合わせる安全確認用(不一致なら中止、--force で無視)。
⚠ CANバスを動かしながらだと USB が不安定になるので、STM32 をリセット保持等でバスを
   黙らせてから実行すること。

参照: odrive_inspect.py(読み取り専用の全体確認), odrive_configure.py(較正),
      tools/hl_control/diagnose_gim_axes.py(CAN経由の状態確認)
"""

import argparse
import json
import re
import sys
import time

import odrive

# apply 時、既定でこのどれかに部分一致するパスだけを対象にする(エンコーダ構造の設定)
DEFAULT_INCLUDE = r"(encoder|mapper|load_encoder|commutation)"

# apply 時、既定でこのどれかに部分一致するパスは除外する(個体固有の較正結果・通信設定)
DEFAULT_EXCLUDE = (
    r"(offset|direction|pre_calibrated|is_calibrated|phase_resistance|phase_inductance"
    r"|calib|serial_number|node_id|\.can\.|hw_version|fw_version|_gpio_pin"
    r"|pos_estimate|vel_estimate|shadow_count|count_in_cpr|spi_error|abs_spi_pos)"
)

MAX_DEPTH = 6


def parse_args():
    p = argparse.ArgumentParser(
        description="動いているGIM6010-8のODrive設定をダンプし、エンコーダ関連だけをGIM8108-8へclone書き込みする")
    p.add_argument("--dump", metavar="FILE", help="接続中の個体の全設定をFILE(JSON)へ書き出す")
    p.add_argument("--show", metavar="FILE", help="ダンプ済みJSONのエンコーダ関連だけを表示して終了(USB不要)")
    p.add_argument("--diff", nargs=2, metavar=("A", "B"),
                    help="設定の差分を表示。A/B はダンプ済みJSONファイル、または 'device'(=いま"
                         "USB接続中の個体をその場で走査)。例: --diff good.json device / "
                         "--diff good.json bad.json。既定は .config. 値のみ比較(--all で全項目、"
                         "ただし毎回変わるライブ値は自動除外)。--include/--exclude も効く")
    p.add_argument("--apply", metavar="FILE", help="ダンプ済みJSONのエンコーダ関連を接続中の個体へ反映する")
    p.add_argument("--write", action="store_true",
                    help="--apply 時、実際に書き込む(付けないとドライラン=差分表示のみ)")
    p.add_argument("--save", action="store_true",
                    help="--apply --write 後に save_configuration() する(ODrive再起動)")
    p.add_argument("--can-id", "--can_id", dest="can_id", type=int, default=None,
                    help="接続中の個体が持っているはずの node_id(安全確認用)")
    p.add_argument("--force", action="store_true", help="node_id 不一致でも続行")
    p.add_argument("--include", default=DEFAULT_INCLUDE,
                    help=f"apply対象パスの部分一致正規表現 (既定: {DEFAULT_INCLUDE})")
    p.add_argument("--exclude", default=DEFAULT_EXCLUDE,
                    help="apply除外パスの部分一致正規表現 (既定は個体固有の較正結果・通信設定)")
    p.add_argument("--all", action="store_true",
                    help="--include を無視して全設定を対象にする(--exclude は効く。上級者向け)")
    return p.parse_args()


def children(obj):
    names = set()
    try:
        names.update(n for n in dir(obj) if not n.startswith("_"))
    except Exception:
        pass
    for attr in ("_remote_attributes", "_properties"):
        try:
            names.update(getattr(obj, attr).keys())
        except Exception:
            pass
    return sorted(names)


def is_leaf(v):
    return isinstance(v, (bool, int, float, str)) or v is None


def walk(obj, prefix="", depth=0, out=None):
    """objの葉ノード(bool/int/float/str)を {パス: 値} で集める。"""
    if out is None:
        out = {}
    if depth >= MAX_DEPTH:
        return out
    for name in children(obj):
        try:
            v = getattr(obj, name)
        except Exception:
            continue
        if callable(v):
            continue
        path = f"{prefix}.{name}" if prefix else name
        if is_leaf(v):
            # bool は int のサブクラスなので先に判定
            if isinstance(v, bool):
                out[path] = bool(v)
            elif isinstance(v, int):
                out[path] = int(v)
            elif isinstance(v, float):
                out[path] = float(v)
            else:
                out[path] = v
        else:
            walk(v, path, depth + 1, out)
    return out


def get_by_path(root, path):
    cur = root
    for part in path.split("."):
        cur = getattr(cur, part)
    return cur


def set_by_path(root, path, value):
    parts = path.split(".")
    parent = root
    for part in parts[:-1]:
        parent = getattr(parent, part)
    setattr(parent, parts[-1], value)


def connect_and_check(can_id, force):
    print("ODriveを検索中... (USB接続を確認してください)")
    odrv0 = odrive.find_any()
    actual = None
    try:
        actual = odrv0.axis0.config.can.node_id
    except AttributeError:
        pass
    fw = ".".join(str(getattr(odrv0, f"fw_version_{k}", "?"))
                  for k in ("major", "minor", "revision"))
    print(f"接続: serial={odrv0.serial_number}, node_id={actual}, fw={fw}")
    if can_id is not None and actual != can_id:
        print(f"\n⚠ node_id({actual}) が --can-id({can_id}) と一致しません。")
        if not force:
            print("誤操作防止のため中止します。--force で続行できます。")
            sys.exit(1)
        print("--force 指定のため続行します。")
    return odrv0


def cmd_dump(path, can_id, force):
    odrv0 = connect_and_check(can_id, force)
    print("全設定ツリーを走査中...")
    values = walk(odrv0)
    meta = {
        "serial_number": str(odrv0.serial_number),
        "node_id": get_safe(odrv0, "axis0.config.can.node_id"),
        "fw_version": ".".join(str(getattr(odrv0, f"fw_version_{k}", "?"))
                                for k in ("major", "minor", "revision")),
        "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"meta": meta, "values": values}, f, indent=2, ensure_ascii=False, sort_keys=True)
    print(f"{len(values)} 項目を {path} に保存しました。")
    show_encoder_subset(values)


def get_safe(root, path, default=None):
    try:
        return get_by_path(root, path)
    except AttributeError:
        return default


def show_encoder_subset(values):
    inc = re.compile(DEFAULT_INCLUDE)
    print("\n--- エンコーダ関連の設定 (.config. のみ) ---")
    cfg = {k: v for k, v in values.items() if inc.search(k) and ".config." in k}
    if not cfg:
        print("  (エンコーダ関連の設定が見つかりませんでした)")
    for k in sorted(cfg):
        print(f"  {k} = {cfg[k]}")

    live = {k: v for k, v in values.items()
            if inc.search(k) and ".config." not in k and not k.startswith("axis0.task_times")}
    if live:
        print("\n--- (参考)エンコーダ関連のライブ値 — clone対象外 ---")
        for k in sorted(live):
            print(f"  {k} = {live[k]}")


def cmd_show(path):
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    print(f"meta: {data.get('meta')}")
    show_encoder_subset(data.get("values", {}))


# --all 時、毎回の読み取りで変動するライブ値(設定ではない)を差分から除外するための正規表現
NOISE_LIVE = re.compile(
    r"(task_times|\.times\.|uptime|timestamp|n_evt|loop_time|"
    r"pos_estimate|vel_estimate|pos_circular|pos_cpr_counts|pos_estimate_counts|pos_abs|"
    r"count_in_cpr|shadow_count|delta_pos|\.phase$|\.phase_vel$|interpolation|calib_scan_response|"
    r"temperature|vbus_voltage|ibus|_measured$|Iq_|Id_|electrical_power|mechanical_power|"
    r"sensorless_estimator\.(pll_pos|vel_estimate|phase)|"
    r"\.is_ready$|\.index_found$|\.hall_state$|spi_error|"
    r"controller\.(pos_setpoint|vel_setpoint|torque_setpoint|input_pos|input_vel|input_torque|"
    r"effective_current_limit|trajectory|autotuning|mechanical_power|electrical_power)|"
    r"current_control\.|\.error$|\.disarm_reason$|current_state$|requested_state$|is_armed$)"
)


def load_diff_side(spec, args):
    """spec が 'device'/'live' なら USB 接続してその場で走査、そうでなければ JSON ファイルとして読む。
    戻り値: (label, meta, values_dict)"""
    if spec.lower() in ("device", "live", "-"):
        odrv0 = connect_and_check(args.can_id, args.force)
        print(f"  ('{spec}') 全ツリーを走査中...")
        values = walk(odrv0)
        meta = {
            "source": "device",
            "node_id": get_safe(odrv0, "axis0.config.can.node_id"),
            "fw_version": ".".join(str(getattr(odrv0, f"fw_version_{k}", "?"))
                                    for k in ("major", "minor", "revision")),
            "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        return f"device({meta['node_id']})", meta, values
    with open(spec, encoding="utf-8") as f:
        data = json.load(f)
    return spec, data.get("meta"), data.get("values", {})


def cmd_diff(spec_a, spec_b, args):
    label_a, meta_a, va = load_diff_side(spec_a, args)
    label_b, meta_b, vb = load_diff_side(spec_b, args)
    print(f"\nA = {label_a}  meta={meta_a}")
    print(f"B = {label_b}  meta={meta_b}")

    # --include は既定値(エンコーダ用)を明示的に上書きした時だけ diff に効かせる
    inc = re.compile(args.include) if (args.include and args.include != DEFAULT_INCLUDE) else None
    exc = re.compile(args.exclude) if (args.exclude and args.exclude != DEFAULT_EXCLUDE) else None

    def want(k):
        if not args.all and ".config." not in k:
            return False
        if args.all and NOISE_LIVE.search(k):
            return False
        if inc is not None and not inc.search(k):
            return False
        if exc is not None and exc.search(k):
            return False
        return True

    keys = sorted(k for k in (set(va) | set(vb)) if want(k))
    MISS = "<なし>"
    changed = only_a = only_b = 0
    scope = "全項目(ライブ値除く)" if args.all else ".config."
    print(f"\n--- {scope} の差分 (A | B) ---")
    for k in keys:
        x, y = va.get(k, MISS), vb.get(k, MISS)
        if x == y:
            continue
        if x is MISS:
            only_b += 1
        elif y is MISS:
            only_a += 1
        else:
            changed += 1
        print(f"  {k}\n      A: {x!r}\n      B: {y!r}")
    total = changed + only_a + only_b
    print(f"\n差分 {total} 項目 (値違い {changed} / Aのみ {only_a} / Bのみ {only_b}) "
          f"/ 比較対象 {len(keys)} 項目")
    if total == 0:
        print("→ 完全に一致。差は設定ではなく、機械/ファーム動作/指令経路の側にある。")


def select_targets(ref_values, include, exclude, all_paths):
    inc = None if all_paths else re.compile(include)
    exc = re.compile(exclude) if exclude else None
    targets = {}
    for k, v in ref_values.items():
        # 設定(不揮発)値だけを対象にする。".config." を含まないパスは pos_abs /
        # interpolation / task_times 等のライブ値なので clone 対象外(--all でも同じ)。
        if ".config." not in k:
            continue
        if inc is not None and not inc.search(k):
            continue
        if exc is not None and exc.search(k):
            continue
        targets[k] = v
    return targets


def apply_config_file(odrv0, path, *, include=DEFAULT_INCLUDE, exclude=DEFAULT_EXCLUDE,
                       all_paths=False, write=False, prompt=True):
    """ダンプ済みJSONから対象パスを選び、odrv0 へ差分を表示(必要なら書き込み)する。
    戻り値: (書き込んだ項目数, 失敗・無視した項目数)。
    odrive_configure.py からも呼べるように独立関数にしてある。save_configuration は呼ばない。"""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    ref_values = data.get("values", {})
    print(f"参照JSON: {data.get('meta')}")

    targets = select_targets(ref_values, include, exclude, all_paths)
    if not targets:
        print("対象パスが0件です。--include / --exclude を見直してください。")
        return 0, 0

    print(f"\n対象 {len(targets)} 項目の差分 (現在値 → 参照値):")
    to_write = []
    for k in sorted(targets):
        ref_v = targets[k]
        try:
            cur_v = get_by_path(odrv0, k)
        except AttributeError:
            print(f"  [skip] {k}: 接続中の個体に存在しないパス")
            continue
        if callable(cur_v) or not is_leaf(cur_v):
            print(f"  [skip] {k}: 葉ノードでない")
            continue
        same = (cur_v == ref_v)
        mark = "  " if same else "→ "
        print(f"  {mark}{k}: {cur_v!r}" + ("" if same else f"  →  {ref_v!r}"))
        if not same:
            to_write.append((k, ref_v))

    if not to_write:
        print("\n差分なし。書き込むものはありません。")
        return 0, 0

    if not write:
        print(f"\nドライランです。実際に書き込むには --write を付けてください "
              f"({len(to_write)} 項目)。")
        return 0, 0

    if prompt:
        ans = input(f"\n{len(to_write)} 項目を書き込みます。よろしいですか? [y/N]: ").strip().lower()
        if ans != "y":
            print("中止しました。")
            return 0, 0

    ok, ng = 0, 0
    for k, v in to_write:
        try:
            set_by_path(odrv0, k, v)
            now = get_by_path(odrv0, k)
            if now == v:
                ok += 1
            else:
                print(f"  ⚠ {k}: 書き込んだが {now!r} のまま(read-only?)")
                ng += 1
        except Exception as e:
            print(f"  ⚠ {k}: 書き込み失敗 {e}")
            ng += 1
    print(f"\n書き込み完了: 成功 {ok} / 失敗・無視 {ng}")
    return ok, ng


def cmd_apply(path, args):
    odrv0 = connect_and_check(args.can_id, args.force)
    ok, _ng = apply_config_file(odrv0, path, include=args.include, exclude=args.exclude,
                                 all_paths=args.all, write=args.write, prompt=True)

    if not args.write:
        return
    if args.save:
        print("save_configuration() します (ODrive再起動)...")
        try:
            odrv0.save_configuration()
        except Exception:
            pass
        print("完了。数秒後に再接続して確認してください。")
    elif ok:
        print("※ --save を付けていないので不揮発化されていません。電源を切ると元に戻ります。")


def main():
    args = parse_args()
    modes = [bool(args.dump), bool(args.show), bool(args.diff), bool(args.apply)]
    if sum(modes) != 1:
        print("--dump / --show / --diff / --apply のいずれか1つを指定してください。")
        sys.exit(1)

    if args.show:
        cmd_show(args.show)
    elif args.diff:
        cmd_diff(args.diff[0], args.diff[1], args)
    elif args.dump:
        cmd_dump(args.dump, args.can_id, args.force)
    else:
        cmd_apply(args.apply, args)


if __name__ == "__main__":
    main()
