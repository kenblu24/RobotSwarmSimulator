To build the documentation, ensure you have the documentation-specific dependencies:

```bash
cd RobotSwarmSimulator
pip install -e .[docs]
```

```bash
cd doc
```

To build the documentation, clone this repository:

* ssh: `git clone git@github.com:kenblu24/RobotSwarmSimulator.git`
* https: `git clone https://github.com/kenblu24/RobotSwarmSimulator.git`

Then, from the `doc/` directory, run the following command to build the documentation:

```bash
./single clean
./single html -E
```

The `-E` forces `sphinx-build` to read all files, even if they have not changed.

For other options, see the [sphinx-build documentation](https://www.sphinx-doc.org/en/master/man/sphinx-build.html#makefile-options).

The documentation will be built in the `doc/build` directory,
and the homepage will be locally available at 
`doc/build/html/index.html`.

The make interface is also available to make multiple targets at once,
but you cannot pass arguments to it via the command line.
Instead, set the `SPHINXOPTS` environment variable to pass arguments to `sphinx-build`.
