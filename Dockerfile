FROM ubuntu:trusty
RUN apt-get update && apt-get -y install python2.7 wget python-dev
RUN wget https://bitbucket.org/khinsen/scientificpython/downloads/ScientificPython-2.9.2.tar.gz
RUN wget https://pypi.python.org/packages/67/ab/41e4b42e0519d868347d2cf1051a05ce0170632039c053dee8ffe8b43b0b/numpy-1.8.2.tar.gz
RUN tar zxvf numpy*
RUN cd num* && python setup.py build && python setup.py install
RUN tar zxvf ScientificPython*
RUN cd Sci* && python setup.py build && python setup.py install
ADD kdtree.py .
#RUN apt-get -y install whitedune
CMD python kdtree.py
