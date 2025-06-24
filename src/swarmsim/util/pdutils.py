import pathlib as pl


def save_df(df, fpath, index=False, **kwargs):
    path = pl.Path(fpath)
    savers = {
        'csv': df.to_csv,
        'tsv': df.to_csv,
        'html': df.to_html,
        'json': df.to_json,
        'markdown': df.to_markdown,
        'md': df.to_markdown,
        'txt': df.to_string,
        'parquet': df.to_parquet,
        'xls': df.to_excel,
        'xlsx': df.to_excel,
    }
    with open(fpath, 'wb') as f:
        func = savers.get(path.suffix.lstrip('.'), df.to_csv)
        func(f, index=index, **kwargs)
