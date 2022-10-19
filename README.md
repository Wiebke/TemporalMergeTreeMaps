# Temporal Merge Tree Maps

This is an Inviwo implementation of Temporal Treemaps for statically visualization scalar fields with a feature-based linearization based on augmented merge trees optimized towards temporal stability, as described in our paper:

Wiebke Köpp, Tino Weinkauf, ["Temporal Merge Tree Maps:A Topology-Based Static Visualization for Temporal Scalar Data"](https://doi.org/10.1109/TVCG.2022.3209387) (IEEE VIS 2022).

## Setup

Temporal Merge Tree Maps have been implemented in [Inviwo](https://inviwo.org/). The setup instructions for Inviwo are briefly summarized below and can also be found in the [Inviwo Github Wiki](https://github.com/inviwo/inviwo/wiki).

| Dependency | Windows (64 bit) | Linux | OSX|
| --- | --- | --- | --- |
| Git | [.exe](https://github.com/git-for-windows/git/releases/download/v2.23.0.windows.1/Git-2.23.0-64-bit.exe)  |  `sudo apt-get install git ` | [.dmg](https://sourceforge.net/projects/git-osx-installer/files/git-2.22.0-intel-universal-mavericks.dmg/download?use_mirror=autoselect)
| CMake >= 3.12  | [.msi](https://github.com/Kitware/CMake/releases/download/v3.15.2/cmake-3.15.2-win64-x64.msi)  |  `sudo apt-get install cmake cmake-qt-gui ` | [.dmg](https://github.com/Kitware/CMake/releases/download/v3.15.2/cmake-3.15.2-Darwin-x86_64.dmg)
| Qt >= 5.12   | [.exe](http://download.qt.io/official_releases/qt/5.12/5.12.4/qt-opensource-windows-x86-5.12.4.exe)<sup>1</sup> | [Qt 5 Install instructions](https://wiki.qt.io/Install_Qt_5_on_Ubuntu)<sup>2</sup> | [.dmg](http://download.qt.io/official_releases/qt/5.12/5.12.4/qt-opensource-mac-x64-5.12.4.dmg)<sup>1</sup>
| C++(17) | [Visual Studio Website](https://visualstudio.microsoft.com/) | `sudo apt-get install build-essential` | [XCode Website](https://developer.apple.com/xcode/)

<sup>1</sup> Select the version for the correct C++ compiler.  
<sup>2</sup> The given download path is outdated. Use a newer version number, e.g. 5.12.4.

* Install the dependencies above.
* Clone this repository, the [Inviwo repository](https://github.com/inviwo/inviwo), the [Inviwo modules repository](https://github.com/inviwo/modules)
<!-- * This code in this repository was last tested with Inviwo version [v0.9.11](https://github.com/inviwo/inviwo/releases/tag/v0.9.11) which can be selected with `git checkout v0.9.11`--> 
* Run `git submodule sync --recursive` and `git submodule update --init --recursive` within *both* the local Inviwo copy and the Inviwo modules copy.
* Generate project files using CMake:
  * Set source and binary destination path.
  * Set all Qt variables.
  * Set MergeTreeMaps as an external module (see below).
* Open the resulting project in Visual Studio/XCode/Shell and build it.

### External Module

The contents of this repository need to be integrated in Inviwo as an external module.
* Add the path to the modules folder of this repository (*TemporalTreeMaps/modules*) to **IVW_EXTERNAL_MODULES** in CMake.
* Select the module **IVW_MODULE_MERGETREEMAPS**.

This will enable some other modules our module depends on as well.

## Examples

Some of the input scalar fields can be found under *modules/mergetreemaps/data/*, sources for the other data sets are listed in the paper.

Workspaces to recreate the visualizations from the paper are located under *modules/mergetreemaps/data/workspaces* and are additionally accessible through the Inviwo user interface under *File → Example Workspaces → MergeTreeMaps*. 

For some of our results, we make use of Python scripts for additional processing (e.g. iterating over parameter settings in the optimization or simplification, repeating the analysis for recording timings) is necessary. These can be found in *modules/mergetreemaps/data/scripts*

<!--### Storms

### Comparisons to other methods

### Evaluation of the optimization

### Evaluation of the simplification

### Other

Some simple examples for scalar fields containing moving and stationary (skewed) Gaussian features are included as well.-->

## Cite

Please cite our paper if you use this code in your own work:

```
@article{KoeppWeinkauf2022_TMTM,
  author    = {Wiebke K\"{o}pp and Tino Weinkauf},
  title     = {Temporal Merge Tree Maps: A Topology-Based Static Visualization for Temporal Scalar Data},
  journal   = {IEEE Transactions on Visualization and Computer Graphics (Proc. IEEE VIS)},
  year      = {2022},
  volume    = {29},
  number    = {1},
  month     = jan,
  doi       = {10.1109/TVCG.2022.3209387},
}
```
