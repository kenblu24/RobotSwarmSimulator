# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'RobotSwarmSimulator'
copyright = '2025, Connor Mattson, Kevin Zhu'
author = 'Connor Mattson, Kevin Zhu'
release = 'latest'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    # 'sphinx.ext.graphviz',
    'sphinx.ext.inheritance_diagram',
    'sphinx.ext.intersphinx',
    'sphinx.ext.ifconfig',
    'sphinx_copybutton',
    'sphinx.ext.todo',
    'sphinx_design',
    # 'sphinx_tags',
    'numpydoc',  # Needs to be loaded *after* autodoc.
    'sphinx.ext.duration',
    "myst_parser",
]

templates_path = ['_templates']
exclude_patterns = []

todo_include_todos = True
copybutton_exclude = '.linenos, .gp, .go'
copybutton_remove_prompts = True
copybutton_only_copy_prompt_lines = True

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
    'scipy': ('https://docs.scipy.org/doc/scipy/', None),
    'numba': ('https://numba.readthedocs.io/en/stable', None),
    'cuquantum': ('https://docs.nvidia.com/cuda/cuquantum/latest', None),
    # blocked by data-apis/array-api#428
    #'array-api': ('https://data-apis.org/array-api/2021.12/', None),
}

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'pydata_sphinx_theme'
html_static_path = ['_static']
html_title = "RobotSwarmSimulator"


html_context = {
    # "github_url": "https://github.com", # or your GitHub Enterprise site
    "github_user": "kenblu24",
    "github_repo": "RobotSwarmSimulator",
    "github_version": "docs",
    "doc_path": "docs/",
}

github_project_url = "https://github.com/kenblu24/RobotSwarmSimulator"


html_theme_options = {
    "use_edit_page_button": True,

    # https://pydata-sphinx-theme.readthedocs.io/en/stable/user_guide/header-links.html
    "icon_links": [
        {
            "name": "GitHub",
            "url": github_project_url,
            "icon": "fa-brands fa-github",
            "type": "fontawesome",
        },
        # {
        #     "name": "PyPI",
        #     "url": pypi_project_url,
        #     "icon": "fa-brands fa-python",
        #     "type": "fontawesome",
        # },
   ],
    # "search_as_you_type": True,
}
# html_show_sourcelink = False
