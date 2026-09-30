#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# generic_logger — logger CSV DÙNG CHUNG (xem GENERIC_LOGGER.md).
#
# Triết lý: node chỉ mô tả "một dòng dữ liệu gồm gì" (dict); thư viện lo suy cột,
# trải vector, ghi CSV. Khác bản gốc: GHI STREAMING (append từng dòng ra đĩa ngay)
# thay vì giữ RAM tới save() -> sống sót kill -9.

import csv
import os


class DataLogger:
    """Ghi dict -> CSV streaming. Tên cột = tên khoá; vector tự trải _0/_1/_2.

    - columns: nếu truyền, CỐ ĐỊNH thứ tự + tên cột (khoá thiếu -> ô trống).
               nếu None, suy từ dòng log() đầu tiên (thứ tự first-seen).
    - Mỗi log() ghi ngay ra đĩa và flush.
    """

    def __init__(self, prefix="log", output_dir=".", columns=None, timestamp=""):
        os.makedirs(output_dir, exist_ok=True)
        name = f"{prefix}_{timestamp}.csv" if timestamp else f"{prefix}.csv"
        self.path = os.path.join(output_dir, name)
        self._columns = list(columns) if columns else None
        self._file = None
        self._writer = None
        self._rows = 0

    @staticmethod
    def _flatten(data):
        """Trải list/tuple/np.array/dict lồng nhau thành các khoá phẳng."""
        flat = {}
        for k, v in data.items():
            if isinstance(v, dict):
                for kk, vv in DataLogger._flatten(v).items():
                    flat[f"{k}_{kk}"] = vv
            elif isinstance(v, (list, tuple)) or hasattr(v, "__len__") and not isinstance(v, str):
                try:
                    for i, vv in enumerate(v):
                        flat[f"{k}_{i}"] = vv
                except TypeError:
                    flat[k] = v
            else:
                flat[k] = v
        return flat

    def _open(self, first_row):
        if self._columns is None:
            self._columns = list(first_row.keys())
        self._file = open(self.path, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=self._columns,
                                      restval="", extrasaction="ignore")
        self._writer.writeheader()
        self._file.flush()

    def log(self, **kwargs):
        row = self._flatten(kwargs)
        if self._writer is None:
            self._open(row)
        self._writer.writerow(row)
        self._file.flush()      # kill -9 safety
        self._rows += 1

    def save(self):
        """Đóng file (dữ liệu đã ghi streaming). Trả về đường dẫn."""
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None
        return self.path

    @property
    def rows(self):
        return self._rows
