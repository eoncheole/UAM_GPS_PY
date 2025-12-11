import pandas as pd
import os
from collections import defaultdict

def analyze_excel_files(directory, file_pattern):
    results = []

    for root, dirs, files in os.walk(directory):
        for file in files:
            if file_pattern.lower() in file.lower() and file.endswith('.xlsx'):
                filepath = os.path.join(root, file)
                try:
                    xl = pd.ExcelFile(filepath)
                    for sheet_name in xl.sheet_names:
                        df = pd.read_excel(filepath, sheet_name=sheet_name)
                        results.append({
                            'file': file,
                            'folder': os.path.basename(root),
                            'sheet': sheet_name,
                            'rows': len(df),
                            'cols': len(df.columns),
                            'columns': list(df.columns),
                            'dtypes': df.dtypes.to_dict()
                        })
                except Exception as e:
                    print(f"Error reading {file}: {e}")
    return results

def compare_structures(results):
    """파일들 간의 구조 비교"""
    by_folder = defaultdict(list)
    for r in results:
        by_folder[r['folder']].append(r)

    for folder, files in by_folder.items():
        print(f"폴더: {folder}")

        unique_columns = set()
        first_cols = tuple(files[0]['columns'])
        all_same = True

        for f in files:
            unique_columns.add(tuple(f['columns']))
            if tuple(f['columns']) != first_cols:
                all_same = False

        print(f"컬럼 구조 일치: {'동일' if all_same else '불일치'}")
        print(f"서로 다른 구조 수: {len(unique_columns)}개")

results = analyze_excel_files(".", "battery")
compare_structures(results)