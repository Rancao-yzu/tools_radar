# -*- mode: python ; coding: utf-8 -*-

a = Analysis(
    ['KPI_SGU_Callback_VW.py'],
    pathex=[],
    binaries=[],
    datas=[],
    hiddenimports=[
        'numpy',
        'numpy._core',
        'numpy._core._dtype_ctypes',
        'pkg_resources',
        'packaging',
        'packaging.version',
        'packaging.specifiers',
        'packaging.requirements',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        # Qt 绑定（特大，你不需要）
        'PyQt5',
        'PySide2',
        'PyQt5.sip',
        'shiboken2',
        
        # 科学计算/大数据（特大，你不需要）
        'scipy',
        'pandas',
        'tensorflow',
        'torch',
        'keras',
        'sklearn',
        'scikit_learn',
        
        # 图像/视觉（特大，你不需要）
        'PIL',
        'Pillow',
        'opencv_python',
        'cv2',
        'matplotlib',
        'plotly',
        
        # Web/网络框架（特大）
        'django',
        'flask',
        'jinja2',
        'tornado',
        'aiohttp',
        'requests',  # ⚠️ 如果用到 HTTP 请求，不要排除
        'urllib3',   # ⚠️ 如果用到，不要排除
        
        # 加密/安全（较大）
        'cryptography',
        'pycryptodome',
        'Crypto',
        'keyring',
        'secretstorage',
        
        # Jupyter/IPython（特大）
        'IPython',
        'jupyter',
        'notebook',
        'ipykernel',
        'jedi',
        'parso',
        'pygments',
        
        # 测试框架（大）
        'pytest',
        'unittest',
        'nose',
        'coverage',
        
        # 数据处理（大）
        'xlrd',
        'openpyxl',
        'xlwt',
        'h5py',
        'netCDF4',
        
        # 数据库（大）
        'sqlalchemy',
        'psycopg2',
        'mysql_connector_python',
        'pymongo',
        
        # 其他大库
        'zmq',
        'pyzmq',
        'beautifulsoup4',
        'soup',
        'Pygments',
        'prompt_toolkit',
        'wcwidth',
        
        # 机器学习相关
        'joblib',
        'threadpoolctl',
        
        # GUI 相关（除了 tkinter）
        'wxPython',
        'PySide6',
        'PyQt6',
        
        # 云服务 SDK（大）
        'boto3',
        'google_cloud',
        'azure',
        
        # 地理信息（大）
        'geopandas',
        'shapely',
        'fiona',
        'pyproj',
    ],
    noarchive=False,
    optimize=0,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='KPI_SGU_Callback_VW',
    debug=False,
    bootloader_ignore_signals=False,
    strip=True,          # 改为 True，减小体积
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)