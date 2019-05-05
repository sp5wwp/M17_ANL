# Project: codec2

CPP      = g++
CC       = gcc
WINDRES  = 
OBJ      = main.o codebook.o codebookd.o codebookdt.o codebookge.o codebookjvm.o codebooklspmelvq.o codebookmel.o codebooknewamp1.o codebooknewamp1_energy.o codebookres.o codebookvq.o codec2.o codec2_fft.o dump.o gp_interleaver.o interldpc.o interp.o kiss_fft.o kiss_fftr.o lpc.o lsp.o mbest.o modem_stats.o mpdecode_core.o newamp1.o nlp.o ofdm.o pack.o phase.o postfilter.o quantise.o sine.o
LINKOBJ  = main.o codebook.o codebookd.o codebookdt.o codebookge.o codebookjvm.o codebooklspmelvq.o codebookmel.o codebooknewamp1.o codebooknewamp1_energy.o codebookres.o codebookvq.o codec2.o codec2_fft.o dump.o gp_interleaver.o interldpc.o interp.o kiss_fft.o kiss_fftr.o lpc.o lsp.o mbest.o modem_stats.o mpdecode_core.o newamp1.o nlp.o ofdm.o pack.o phase.o postfilter.o quantise.o sine.o
LIBS     = -static-libgcc -lm
INCS     = 
CXXINCS  = 
BIN      = m17_anl
CXXFLAGS = $(CXXINCS) -march=native -msse3 -O2 -std=gnu99
CFLAGS   = $(INCS) -march=native -msse3 -O2 -std=gnu99
RM       = rm -f

.PHONY: all all-before all-after clean clean-custom

all: all-before $(BIN) all-after

clean: clean-custom
	${RM} $(OBJ) $(BIN)

$(BIN): $(OBJ)
	$(CC) $(LINKOBJ) -o $(BIN) $(LIBS)

main.o: main.c
	$(CC) -c main.c -o main.o $(CFLAGS)

codebook.o: codebook.c
	$(CC) -c codebook.c -o codebook.o $(CFLAGS)

codebookd.o: codebookd.c
	$(CC) -c codebookd.c -o codebookd.o $(CFLAGS)

codebookdt.o: codebookdt.c
	$(CC) -c codebookdt.c -o codebookdt.o $(CFLAGS)

codebookge.o: codebookge.c
	$(CC) -c codebookge.c -o codebookge.o $(CFLAGS)

codebookjvm.o: codebookjvm.c
	$(CC) -c codebookjvm.c -o codebookjvm.o $(CFLAGS)

codebooklspmelvq.o: codebooklspmelvq.c
	$(CC) -c codebooklspmelvq.c -o codebooklspmelvq.o $(CFLAGS)

codebookmel.o: codebookmel.c
	$(CC) -c codebookmel.c -o codebookmel.o $(CFLAGS)

codebooknewamp1.o: codebooknewamp1.c
	$(CC) -c codebooknewamp1.c -o codebooknewamp1.o $(CFLAGS)

codebooknewamp1_energy.o: codebooknewamp1_energy.c
	$(CC) -c codebooknewamp1_energy.c -o codebooknewamp1_energy.o $(CFLAGS)

codebookres.o: codebookres.c
	$(CC) -c codebookres.c -o codebookres.o $(CFLAGS)

codebookvq.o: codebookvq.c
	$(CC) -c codebookvq.c -o codebookvq.o $(CFLAGS)

codec2.o: codec2.c
	$(CC) -c codec2.c -o codec2.o $(CFLAGS)

codec2_fft.o: codec2_fft.c
	$(CC) -c codec2_fft.c -o codec2_fft.o $(CFLAGS)

dump.o: dump.c
	$(CC) -c dump.c -o dump.o $(CFLAGS)

gp_interleaver.o: gp_interleaver.c
	$(CC) -c gp_interleaver.c -o gp_interleaver.o $(CFLAGS)

interldpc.o: interldpc.c
	$(CC) -c interldpc.c -o interldpc.o $(CFLAGS)

interp.o: interp.c
	$(CC) -c interp.c -o interp.o $(CFLAGS)

kiss_fft.o: kiss_fft.c
	$(CC) -c kiss_fft.c -o kiss_fft.o $(CFLAGS)

kiss_fftr.o: kiss_fftr.c
	$(CC) -c kiss_fftr.c -o kiss_fftr.o $(CFLAGS)

lpc.o: lpc.c
	$(CC) -c lpc.c -o lpc.o $(CFLAGS)

lsp.o: lsp.c
	$(CC) -c lsp.c -o lsp.o $(CFLAGS)

mbest.o: mbest.c
	$(CC) -c mbest.c -o mbest.o $(CFLAGS)

modem_stats.o: modem_stats.c
	$(CC) -c modem_stats.c -o modem_stats.o $(CFLAGS)

mpdecode_core.o: mpdecode_core.c
	$(CC) -c mpdecode_core.c -o mpdecode_core.o $(CFLAGS)

newamp1.o: newamp1.c
	$(CC) -c newamp1.c -o newamp1.o $(CFLAGS)

nlp.o: nlp.c
	$(CC) -c nlp.c -o nlp.o $(CFLAGS)

ofdm.o: ofdm.c
	$(CC) -c ofdm.c -o ofdm.o $(CFLAGS)

pack.o: pack.c
	$(CC) -c pack.c -o pack.o $(CFLAGS)

phase.o: phase.c
	$(CC) -c phase.c -o phase.o $(CFLAGS)

postfilter.o: postfilter.c
	$(CC) -c postfilter.c -o postfilter.o $(CFLAGS)

quantise.o: quantise.c
	$(CC) -c quantise.c -o quantise.o $(CFLAGS)

sine.o: sine.c
	$(CC) -c sine.c -o sine.o $(CFLAGS)
