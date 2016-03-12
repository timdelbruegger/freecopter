# Freecopter - An autonomous Quadrocopter project powered by a Raspberry Pi2

This is the place where I implement my very own autonomous quadrocopter. You can find current development information and history in my blog: https://timdelbruegger.wordpress.com/

If you want to checkout the project, be sure to use the "--recursive" flag so that the submodules with are initialized, too.
git clone --recursive https://github.com/timdelbruegger/freecopter.git

After checkout, you will want to install the requirements:
pip install -r requirements.txt

### Folders:
- *modelling:* mathematical ideas that support the source code.
- *hardware_checks:* Scripts that check the functionality of individual hardware components (normally in Python 3).
- *lib:* other libraries that I use as git submodules.