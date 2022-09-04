# Codeball 2018

Franz Cesista's finals-qualifying strategy for the Russian AI Cup 2018 Codeball competition.

![](goal.png)

## How to play

### Clone the repository

First, clone the repository by running

```bash
git clone https://github.com/leloykun/codeball-2018.git
```

Then go to the folder

```bash
cd codeball-2018
```

### Install the required system libraries

> Note: the simulators work best on Linux. You can run them on Windows, but you'll need to install the [Windows Subsystem for Linux](https://docs.microsoft.com/en-us/windows/wsl/install) first.

Install linux system requirements by running

```bash
cd codeball2018-linux
./install_requirements.sh
```

You may need to giver permission to execute the script by running

```bash
chmod +x install_requirements.sh
```

### Run the game

To play the game against my bot, simply run

```bash
./run.sh
```

Then click on the `(?)` button to read the instructions.

If the current strategy is too hard for you, you can play with an 'easy' strategy by running

```bash
./run.sh helper
```

You can also practice moving around the map by running

```bash
./run.sh empty
```

## Building Your Own Bot

### Code your strategy

First, read the [Developer Guide](developer_guide.pdf). The Developer Guide contains the rules of the game, the API, and the strategy that you need to implement.

Then, modify the contents of `cpp-cgdk/MyStrategy.cpp` to implement your own strategy. You can also modify the associated c++ source files or even add new ones in the `cpp-cgdk` directory if you need to.

### Build the executable file

Once you're done, build the executable file of your bot by running

```bash
./rebuild.sh
```

### Testing

After that, you can run the game and play with your bot by running

```bash
./run.sh
```

You can also run the game with your bot and my pre-made bot by running

```bash
./run.sh auto
```

To run a benchmark test, simply run

```bash
./run_benchmark.sh
```

## Miscellaneous

### Script options (and default values)

For **`run.sh`**:

```bash
./run.sh [GAME_MODE="play"] [IP_ADDRESS="127.0.0.1"] [DURATION=18000] \
[P1_STRATEGY="cpp-cgdk/versions/CesistaStrategy_v47"] [P2_STRATEGY="cpp-cgdk/build/MyStrategy"] \
[P1_NAME="Cesista's Strategy"] [P2_NAME="Current Strategy"] [P1_PORT=31001] \
[P2_PORT=31002] [P1_KEY=0000000000000000] [P2_KEY=0000000000000000]
```

Note: `GAME_MODE` can only be one of `play`, `auto`, `helper`, or `empty`.

For **`run_benchmark.sh`**:

```bash
./run_benchmark [BATCHES=1] [IP_ADDRESS="127.0.0.1"] [DURATION=18000] 
[P1_STRATEGY="cpp-cgdk/versions/CesistaStrategy_v47"] [P2_STRATEGY="cpp-cgdk/build/MyStrategy"] \
[P1_NAME="Cesista's Strategy"] [P2_NAME="Current Strategy"]
```

### FAQ

- Why does the GUI in WSL2 just display a blank canvas?
  - You need to restart your WSL2. Try running `wsl --shutdown` then `wsl`
- Can I share my own bot? If so, how?
  - Yes, of course! Just copy the executable file of your bot to `cpp-cgdk/versions` and rename it to `[your last name here]_v[version number]`.
- How do I play the game?
  - Run the game and click the `(?)` button on the lower right side of the game window.

## How to Contribute

1. Fork this repository.
2. Clone this repository into your local machine.
3. Add and commit your changes to the repository. And don't forget to add your name to the Contributors section below.
4. Submit a Pull Request and tag one of the contributors to review your code.
5. Wait for the review and address the reviewer's comments if necessary.
6. Wait for the reviewer to approve your PR.
7. Merge your PR.

## Contributors

### Main Contributor

- [Franz Louis Cesista](https://github.com/leloykun), ML SWE at [Expedock](https://github.com/expedock), 2x IOI & ICPC World Finalist.
