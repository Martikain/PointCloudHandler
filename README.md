PointCloudHandler
===================
A command line project for basic point cloud operations such file conversion from
CSV to PCD.

## Usage:

File conversion:

You can convert any CSV file containing x, y and z coordinates to PCD format
by using the command 'convert'. Source file path is a required value. The
options for the command are

-s or --source		Source file path (CSV file)

-d or --destination	Destination file path (PCD file)

--delimiter		Delimiter of the source CSV file. Space is the default
			delimiter.

-f or --format		Format of the PCD file. Either ASCII or BINARY.

-i or --intensity	If provided, the converter will parse the first row
			of the CSV file and search for the header given with
			this option. This value will be stored as the intensity
			value in the destination PCD file.

Example: ./PointCloudHandler convert -s /home/username/pointCloud.csv -d /home/username/pointCloud.pcd --delimiter ";" -i intensity

---
