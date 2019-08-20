## Maya Harmonic Deforme

The tcHarmonciDefomer is a free form deformer that uses a closed mesh (cage) to deform an higher resolution mesh. Its implementation is based on the Pixar siggraph paper:
http://graphics.pixar.com/library/HarmonicCoordinatesB/paper.pdf

If you are planning to use one of our tools in a studio, we would be grateful if you could let us know.

## How to use

Select the object that you want deform and then the cage geometry.
From the Toolchefs menu select Deformers->tcHamonicDeformer to apply the deformer. Or you can use the following mel command:
tcCreateHarmonicDeformer
Open the attribute editor and go to the tcHarmonicDeformer tab that was just created. Choose the cell size and the number of iterations, then press the button “Compute Harmonic Weights”.  You can also compute the harmonic weights with the following mel command:
tcComputeHarmonicWeights -d (deformerNode) -mi (iterations) -cs (cellsize) -ts (threshold) -sg (saveGridData)
For instance: tcComputeHarmonicWeights -d tcHarmonicDeformer1 -mi 30 -cs 0.5 -ts 0.00001 -sg 0;
 

# Deformer attributes

* Cell size

The deformer create a 3 dimension grid to voxelize the cage and solve the laplace equation inside this grid in order to diffuse the weights of the vertices. The cell size is the dimension of the cell. If you decrease the cell size, the weights will be more smooth but more localized near the cage.
The smaller the cell size the greater the weight computation time will be. It will also use more hardware resources.
Please use this value carefully taking into account the sizes of your mesh and cage.

* Iterations

The deformer solves the Laplace equation using an iterative method. If you increase the iterations the weights spreads more evenly inside the cage.
The greater this value is the greater the weight computation time will be. It will also use more hardware resources.

* Threshold

The weights that have value less than the threshold will not be stored inside the deformer.
The default value is 0.000001.

* Dynamic binding

When the “dynamic binding” is on, the deformer queries a pre-solved grid and reassigns the weights to the input model whenever it changes. To make this happen, you must compute the weights with the “dynamicBinding” attribute on or set the sg flag of the tcComputeHarmonciWeights command to 1.
Just be careful as this will use much more memory and your scene file size will be a lot bigger, since all the weights for every grid cell will be stored inside the deformer.

##Know limitations:

The cage must be a closed mesh.
The cage must have only faces with 3 or 4 vertices.

## License

This project is licensed under [the LGPL license](http://www.gnu.org/licenses/).

## Contact us

Please feel free to contact us at support@toolchefs.com in case you would like contribute or simply have questions.

### ENJOY!
