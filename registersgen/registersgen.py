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
import argparse

@dataclass
class Field:
    name: str
    width: str
    default: str
    descr: str

@dataclass
class Register:
    name: str
    address: str
    max_field_strlen: int
    fields: List[Field]

datasheet_url = "https://www.st.com/resource/en/datasheet/l9963e.pdf"
file = datasheet_url

dir = Path(__file__).parent
header = j2.Template((dir / "registers.h.j2").read_text())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simple script that generates header files from the L9963E datasheet")
    parser.add_argument("output", type=str, help="output filename")
    parser.add_argument("-F", "--force", action='store_true', help="overwrite existing output file", required=False)
    parser.add_argument("-f", "--file", type=str, help="datasheet path or URL, defaults to " + datasheet_url, required=False)

    args = parser.parse_args()

    if args.file:
        file = args.file

    outputfile = Path(str(args.output))

    if not args.force and outputfile.exists():
        print(args.output + " already exists. Use -F or --force to overwrite it")
        exit()

    tables = tabula.read_pdf(file, pages=[*range(102, 128)], lattice=True)

    t = pd.concat(tables[1:])

    t['descr'] = t['Reset value\reset sources\rDescription'].fillna("") + t['Description'].fillna("")
    t = t.rename(columns={'Register Name': 'regname', 'Unnamed: 0': 'address', 'Field name': 'fieldname', 'Unnamed: 3': 'fieldwidth', 'Unnamed: 4': 'field_default'})
    t = t.filter(items=['regname', 'address', 'fieldname', 'fieldwidth', 'field_default', 'descr'])

    # print(t)
    # exit()

    registers = []

    max_reg_strlen = 0

    last_descr = ""
    for _, row in t.iterrows():
        if not row['regname'] != row['regname']:
            if len(registers) > 0 and len(registers[-1].fields) > 0 and "Noreg" not in registers[-1].fields[-1].name:
                registers[-1].fields[-1].descr = last_descr
                
            max_reg_strlen = max(len(row['regname'].replace('\r', ''))+4, max_reg_strlen)
            registers.append(Register(row['regname'].replace('\r', ''), row['address'], 0, []))
        elif not (row['fieldname'] != row['fieldname'] or row['fieldwidth'] != row['fieldwidth'] or row['field_default'] != row['field_default']):
            if len(registers[-1].fields) > 0:
                if ("Noreg" in row['fieldname'] or row['descr'] != "") and "Noreg" not in registers[-1].fields[-1].name:
                    registers[-1].fields[-1].descr = last_descr

            prov_descr = "\r/* Reserved */" if "Noreg" in row['fieldname'] else ""
            registers[-1].max_field_strlen = max(registers[-1].max_field_strlen, len(str(row['fieldname']).replace('\r', ''))+4)
            registers[-1].fields.append(Field(str(row['fieldname']).replace('\r', ''), int(row['fieldwidth']), row['field_default'], prov_descr))

        if row['descr'] != "":
            last_descr = "\r/* " + row['descr'] + " */"


    for i, reg in enumerate(registers):
        if sum([f.width for f in reg.fields]) != 18:
            print(f"Error on register {reg.name}")
        registers[i].fields = reg.fields[::-1]

    args = {
        "registers": registers,
        "max_reg_strlen": max_reg_strlen
    }

    outputfile.write_text(header.render(**args))
