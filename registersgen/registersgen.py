# 
# "THE BEER-WARE LICENSE" (Revision 69):
# Squadra Corse firmware team wrote this file. As long as you retain this notice
# you can do whatever you want with this stuff. If we meet some day, and you 
# think this stuff is worth it, you can buy us a beer in return.
# 
# Authors
# - Federico Carbone [federico.carbone.sc@gmail.com]
# 

import tabula
import pandas as pd
from dataclasses import dataclass
from typing import List
import jinja2 as j2
from pathlib import Path

@dataclass
class Field:
    name: str
    width: str
    default: str

@dataclass
class Register:
    name: str
    address: str
    max_field_strlen: int
    fields: List[Field]

file = "https://www.st.com/resource/en/datasheet/l9963e.pdf"

dir = Path(__file__).parent
header = j2.Template((dir / "registers.h.j2").read_text())

tables = tabula.read_pdf(file, pages=[*range(102, 128)], lattice=True)

t = pd.concat(tables[1:])

t = t.rename(columns={'Register Name': 'regname', 'Unnamed: 0': 'address', 'Field name': 'fieldname', 'Unnamed: 3': 'fieldwidth', 'Unnamed: 4': 'field_default', 'Description': 'descr'})

registers = []

max_reg_strlen = 0

for _, row in t.iterrows():
    if not row['regname'] != row['regname']:
        max_reg_strlen = max(len(row['regname'].replace('\r', ''))+4, max_reg_strlen)
        registers.append(Register(row['regname'].replace('\r', ''), row['address'], 0, []))
    elif not (row['fieldname'] != row['fieldname'] or row['fieldwidth'] != row['fieldwidth'] or row['field_default'] != row['field_default']):
        registers[-1].max_field_strlen = max(registers[-1].max_field_strlen, len(str(row['fieldname']).replace('\r', ''))+4)
        registers[-1].fields.append(Field(str(row['fieldname']).replace('\r', ''), int(row['fieldwidth']), row['field_default']))

for i, reg in enumerate(registers):
    if sum([f.width for f in reg.fields]) != 18:
        print(f"Error on register {reg.name}")
    registers[i].fields = reg.fields[::-1]

args = {
    "registers": registers,
    "max_reg_strlen": max_reg_strlen
}

cwd = Path.cwd()

(cwd / "L9963E_registers.h").write_text(header.render(**args))