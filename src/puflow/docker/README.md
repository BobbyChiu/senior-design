
## Steps to start

```bash
git clone https://github.com/unknownue/puflow.git
cd puflow/docker
wsl -e DOCKER_BUILDKIT=0 docker build -t unknownue/nf -f Dockerfile .

# for cmd, if using bash replace %cd% with $(pwd)

docker run -id -v %cd%:/workspace --rm --gpus all --name nf -e NVIDIA_DRIVER_CAPABILITIES=graphics,display,compute,utility -w /workspace --shm-size 8G unknownue/nf
docker exec -it -w /workspace/ nf bash
```


## Issues
1. Docker must be able to access the GPU during build
### Fix
Add the following to your daemon.json:
```json
{
    "runtimes": {
        "nvidia": {
            "path": "/usr/bin/nvidia-container-runtime",
            "runtimeArgs": []
         } 
    },
    "default-runtime": "nvidia" 
}

If experiences memory issues. Try to 
1. Modify memory= in .wslconfig

Make sure when you run the build that it confirms your are running with buildkit disabled.
If it is not disabled, then you must perform the build within wsl by running:

```bash
wsl -e DOCKER_BUILDKIT=0 docker build <blah>
```
2. Make sure cuda version used to compile pytorch extensions is the same as the one running in the container
3. metrics/PyTorchCD is a submodule that needs to be pulled separetely
ls
