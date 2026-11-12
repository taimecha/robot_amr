#!/usr/bin/env bash
# The historical alignment command is retired. This file never starts hardware.
set -euo pipefail
echo 'Bộ căn cũ đã ngừng dùng. Lệnh này không chạy quỹ đạo A → C → B và sẽ không khởi động xe.' >&2
echo 'Chưa có launcher chạy thật A → C → B; cần hoàn tất hiệu chuẩn và bộ điều khiển thật.' >&2
exit 2
