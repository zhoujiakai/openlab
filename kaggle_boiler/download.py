"""下载锅炉数据集，保留已有的本地文件。"""
from pathlib import Path
import shutil
import kagglehub


def main():
    destination = Path(__file__).resolve().parent / 'data'
    if destination.exists():
        print(f'数据目录已存在，跳过下载：{destination}')
        return
    source = kagglehub.dataset_download('nikitamanaenkov/time-series-of-industrial-boiler-operations')
    shutil.copytree(source, destination)
    print(f'数据已保存：{destination}')


if __name__ == '__main__':
    main()
