#!/usr/bin/env python3
"""kpi_logger CSV → 추종거리 p95, Lost/재획득 시간, IPH, 배송 성공률 요약. TODO([웹캠]) — BRD v3.1 5.3 정의대로."""
import sys
import csv

path = sys.argv[1]
rows = list(csv.DictReader(open(path, encoding='utf-8')))
print(f'{len(rows)} rows')
# TODO: source=='intervention' 개수 / 운용시간 → IPH ; source=='hint' 와 fleet x,y 로 추종거리 ; mission state 전이로 리드타임
