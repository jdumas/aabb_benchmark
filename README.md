# AABB Tree Benchmark

### Data

You can place additional downloaded models in `data/`.
`bunny.off` is provided for your convenience.
Here are some suggestions:

- [Stanford dragon](https://cs.nyu.edu/courses/spring18/CSCI-GA.2270-001/data/dragon.off)
- [Thingi10K](https://ten-thousand-models.appspot.com/)

### Compilation

Make sure to compile in release!

```
mkdir build; cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j 8
```

### Benchmark

To display statistics about the results, you need to have [pandas](https://pandas.pydata.org/) available.

```
cd python
./benchmark.py -c distance_to_mesh -o dist.json ../data
./benchmark.py -c ray_tracing -o ray.json ../data
```

Output statistics:

```
./stats.py dist.json 
                count      mean       std       min       25%       50%       75%       max
method                                                                                     
geogram          17.0  0.122634  0.090985  0.018138  0.057599  0.085547  0.213004  0.284186
morton_binary    17.0  0.179299  0.136215  0.039204  0.072267  0.117081  0.303362  0.457230
hilbert_binary   17.0  0.179900  0.137642  0.038078  0.071600  0.116555  0.316527  0.454705
hilbert          17.0  0.187877  0.146730  0.033257  0.072034  0.121204  0.321534  0.477837
ours_binary      17.0  0.210401  0.173534  0.020379  0.097633  0.133029  0.316605  0.591937
ours             17.0  0.222995  0.181206  0.028541  0.100343  0.140565  0.334233  0.594097
morton           17.0  0.240368  0.211510  0.022894  0.108532  0.164132  0.406410  0.784553
igl              17.0  0.293843  0.272498  0.032069  0.110822  0.154889  0.475230  0.914263
```

```
./stats.py ray.json 
                count      mean       std       min       25%       50%       75%        max
method                                                                                      
embree           17.0  0.247782  0.102379  0.084333  0.190652  0.219816  0.295780   0.423347
geogram          17.0  0.637124  0.332892  0.124273  0.369079  0.658865  0.826838   1.360295
ours             17.0  0.685104  0.386073  0.182225  0.357059  0.748395  0.880797   1.489862
morton           17.0  0.719434  0.356301  0.156949  0.424598  0.750109  0.963619   1.465881
hilbert          17.0  0.732410  0.377974  0.154870  0.428783  0.749345  0.965261   1.558961
ours_binary      17.0  0.965989  0.574602  0.239528  0.491594  0.995952  1.220360   2.237329
hilbert_binary   17.0  1.037296  0.547904  0.200241  0.575182  1.047844  1.337041   2.173295
morton_binary    17.0  1.052529  0.563368  0.203320  0.576216  1.057456  1.339751   2.178193
igl              17.0  3.528063  3.216025  0.501061  1.010425  3.157664  5.004837  13.671037
```
