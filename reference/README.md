# 移植元の未変更clone

`stm32_board_pid_pos_vel_control_set/` は、ユーザー指定のリポジトリを
そのまま `git clone --depth 1` した参照用チェックアウトです。

- Origin: https://github.com/TNCT-Mechatech/stm32_board_pid_pos_vel_control_set.git
- Commit: `78edb8b795f9315d4492b7b9716ea78b12200a4f`
- カスタマイズ先: `../src/stm32_pos_control_test/`

このディレクトリの `COLCON_IGNORE` により、元のパッケージや同梱ビルド成果物は
このワークスペースのcolconビルドに含まれません。
clone本体はルートの `.gitignore` で除外し、作業コピーだけを変更しています。

別の端末で参照cloneを再取得する場合は、ワークスペースのルートで次を実行します。

```bash
git clone https://github.com/TNCT-Mechatech/stm32_board_pid_pos_vel_control_set.git reference/stm32_board_pid_pos_vel_control_set
git -C reference/stm32_board_pid_pos_vel_control_set checkout 78edb8b795f9315d4492b7b9716ea78b12200a4f
```
