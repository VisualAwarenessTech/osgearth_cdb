// Copyright (c) 2016-2017 Visual Awareness Technologies and Consulting Inc, St Petersburg FL
// This file is based on the Common Database (CDB) Specification for USSOCOM
// Version 3.0 – October 2008

// CDB_Tile is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// CDB_Tile is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public License
// along with CDB_Tile.  If not, see <http://www.gnu.org/licenses/>.

// 2015 GAJ Geospatial Enterprises, Orlando FL
// Modified for General Incorporation of Common Database (CDB) support within osgEarth
//
#include "CDB_Tile"

#ifdef _WIN32
#include <Windows.h>
#endif

#define GEOTRSFRM_TOPLEFT_X            0
#define GEOTRSFRM_WE_RES               1
#define GEOTRSFRM_ROTATION_PARAM1      2
#define GEOTRSFRM_TOPLEFT_Y            3
#define GEOTRSFRM_ROTATION_PARAM2      4
#define GEOTRSFRM_NS_RES               5

#define JP2DRIVERCNT 5

CDB_GDAL_Drivers Gbl_TileDrivers;

static int s_BaseMapLodNum = 0;
static bool s_EnableBathymetry = true;

const int Gbl_CDB_Tile_Sizes[11] = {1024, 512, 256, 128, 64, 32, 16, 8, 4, 2, 1};
//Caution this only goes down to CDB Level 17
const double Gbl_CDB_Tiles_Per_LOD[18] = {1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 128.0, 256.0, 512.0, 1024.0, 2048.0, 4096.0, 8192.0, 16384.0, 32768.0, 65536.0, 131072.0};

OGR_File  Ogr_File_Instance;

CDB_Tile::CDB_Tile(std::string cdbRootDir, std::string cdbCacheDir, CDB_Tile_Type TileType, std::string dataset, CDB_Tile_Extent *TileExtent, int NLod) : m_cdbRootDir(cdbRootDir), m_cdbCacheDir(cdbCacheDir),
				   m_DataSet(dataset), m_TileExtent(*TileExtent), m_TileType(TileType), m_ImageContent_Status(NotSet), m_Tile_Status(Created), m_FileName(""), m_LayerName(""), m_FileExists(false),
				   m_CDB_LOD_Num(0), m_Subordinate_Exists(false), m_SubordinateName(""), m_lat_str(""), m_lon_str(""), m_lod_str(""), m_uref_str(""), m_rref_str(""), m_Subordinate_Tile(false)
{
	m_GTModelSet.clear();

	if (NLod > 0)
	{
		m_Pixels.pixX = Gbl_CDB_Tile_Sizes[NLod];
		m_Pixels.pixY = Gbl_CDB_Tile_Sizes[NLod]; 
		m_CDB_LOD_Num = -NLod;
	}

	std::stringstream	buf;
	std::stringstream	subordinatebuf;

	m_CDB_LOD_Num = GetPathComponents(m_lat_str, m_lon_str, m_lod_str, m_uref_str, m_rref_str);

	std::string filetype;
	std::string datasetstr;
	std::string subordinatedatasetstr;

	if (m_TileType == Elevation)
	{
		m_LayerName = "001_Elevation";
		if (m_CDB_LOD_Num < 0 && NLod == 0)
		{
			m_TileType = ElevationCache;
			filetype = ".img";
		}
		else
		{
			filetype = ".tif";
		}

		datasetstr = "_D001" + m_DataSet;
		subordinatedatasetstr = "_D001_S100_T001_";
		m_Pixels.pixType = GDT_Float32;
		m_Pixels.bands = 1;
	}
	else if (m_TileType == Imagery)
	{
		m_LayerName = "004_Imagery";
		if (m_CDB_LOD_Num < 0 && NLod == 0)
		{
			m_TileType = ImageryCache;
			filetype = ".tif";
		}
		else
		{
			filetype = ".jp2";
		}
		datasetstr = "_D004" + m_DataSet;
	}
	else if (m_TileType == GeoPackageMap)
	{
		m_LayerName = "901_VectorBase";
		if (m_CDB_LOD_Num < s_BaseMapLodNum)
			filetype = ".xml";
		else
			filetype = ".gpkg";
		datasetstr = "_D901" + m_DataSet;
	}
	else if (m_TileType == GeoSpecificModel)
	{
		m_LayerName = "100_GSFeature";	
		filetype = ".shp";
		datasetstr = "_D100" + m_DataSet;
	}
	else if (m_TileType == GeoTypicalModel)
	{
		m_LayerName = "101_GTFeature";	
		filetype = ".shp";
		datasetstr = "_D101" + m_DataSet;
	}
	else
	{
		m_TileType = CDB_Unknown;
		filetype = ".unk";
		datasetstr = "_DUNK_SUNK_TUNK_";
	}

	//Set tile size for lower levels of detail

	if (m_CDB_LOD_Num < 0)
	{
		if (NLod == 0)
		{
			buf << cdbCacheDir
				<< "\\" << m_LayerName
				<< "\\" << m_lat_str << m_lon_str << datasetstr << m_lod_str
				<< "_" << m_uref_str << "_" << m_rref_str << filetype;

			if ((m_TileType == Elevation) || (m_TileType == ElevationCache))
			{
				subordinatebuf << cdbRootDir
					<< cdbCacheDir
					<< "\\" << m_LayerName
					<< "\\" << m_lat_str << m_lon_str << subordinatedatasetstr << m_lod_str
					<< "_" << m_uref_str << "_" << m_rref_str << filetype;
			}
		}
		else
		{
			buf << cdbRootDir
				<< "\\Tiles"
				<< "\\" << m_lat_str
				<< "\\" << m_lon_str
				<< "\\" << m_LayerName
				<< "\\LC"
				<< "\\" << m_uref_str
				<< "\\" << m_lat_str << m_lon_str << datasetstr << m_lod_str
				<< "_" << m_uref_str << "_" << m_rref_str << filetype;
			if ((m_TileType == Elevation) || (m_TileType == ElevationCache))
			{
				subordinatebuf << cdbRootDir
					<< "\\Tiles"
					<< "\\" << m_lat_str
					<< "\\" << m_lon_str
					<< "\\" << m_LayerName
					<< "\\LC"
					<< "\\" << m_uref_str
					<< "\\" << m_lat_str << m_lon_str << subordinatedatasetstr << m_lod_str
					<< "_" << m_uref_str << "_" << m_rref_str << filetype;

			}
		}
	}
	else
	{
		buf << cdbRootDir
			<< "\\Tiles"
			<< "\\" << m_lat_str
			<< "\\" << m_lon_str
			<< "\\" << m_LayerName
			<< "\\" << m_lod_str
			<< "\\" << m_uref_str
			<< "\\" << m_lat_str << m_lon_str << datasetstr << m_lod_str
			<< "_" << m_uref_str << "_" << m_rref_str << filetype;

		if ((m_TileType == Elevation) || (m_TileType == ElevationCache))
		{
			subordinatebuf << cdbRootDir
				<< "\\Tiles"
				<< "\\" << m_lat_str
				<< "\\" << m_lon_str
				<< "\\" << m_LayerName
				<< "\\" << m_lod_str
				<< "\\" << m_uref_str
				<< "\\" << m_lat_str << m_lon_str << subordinatedatasetstr << m_lod_str
				<< "_" << m_uref_str << "_" << m_rref_str << filetype;
		}
	}
	m_FileName = buf.str();

	if (m_TileType == GeoSpecificModel)
	{
		std::string dataset2str = "_D100_S001_T002_";
		std::string filetype2str = ".dbf";
		std::stringstream dbfbuf;

		dbfbuf << cdbRootDir
			<< "\\Tiles"
			<< "\\" << m_lat_str
			<< "\\" << m_lon_str
			<< "\\" << m_LayerName
			<< "\\" << m_lod_str
			<< "\\" << m_uref_str
			<< "\\" << m_lat_str << m_lon_str << dataset2str << m_lod_str
			<< "_" << m_uref_str << "_" << m_rref_str << filetype2str;

		m_ModelSet.ModelDbfName = dbfbuf.str();

		dataset2str = "_D300_S001_T001_";
		filetype2str = ".zip";
		std::string layerName2 = "300_GSModelGeometry";
		std::stringstream geombuf;
		geombuf << cdbRootDir
			<< "\\Tiles"
			<< "\\" << m_lat_str
			<< "\\" << m_lon_str
			<< "\\" << layerName2
			<< "\\" << m_lod_str
			<< "\\" << m_uref_str
			<< "\\" << m_lat_str << m_lon_str << dataset2str << m_lod_str
			<< "_" << m_uref_str << "_" << m_rref_str << filetype2str;

		m_ModelSet.ModelGeometryName = geombuf.str();

		dataset2str = "_D301_S001_T001_";
		filetype2str = ".zip";
		layerName2 = "301_GSModelTexture";
		std::stringstream txbuf;
		txbuf << cdbRootDir
			<< "\\Tiles"
			<< "\\" << m_lat_str
			<< "\\" << m_lon_str
			<< "\\" << layerName2
			<< "\\" << m_lod_str
			<< "\\" << m_uref_str
			<< "\\" << m_lat_str << m_lon_str << dataset2str << m_lod_str
			<< "_" << m_uref_str << "_" << m_rref_str << filetype2str;

		m_ModelSet.ModelTextureName = txbuf.str();

		int Tnum = 1;
		int i = 1;
		std::stringstream laybuf;
		laybuf << "100_GSFeature_S" << std::setfill('0') << std::setw(3) << abs(i) << "_T"
			<< std::setfill('0') << std::setw(3) << abs(Tnum) << "_Pnt";
		m_ModelSet.PrimaryLayerName = laybuf.str();

		std::stringstream clslaybuf;
		clslaybuf << "100_GSFeature_S" << std::setfill('0') << std::setw(3) << abs(i) << "_T"
			<< std::setfill('0') << std::setw(3) << abs(Tnum + 1) << "_Cls";
		m_ModelSet.ClassLayerName = clslaybuf.str();

	}
	else if (m_TileType == GeoTypicalModel)
	{
		int Tnum = 1;
		std::string filetype2str = ".dbf";

		for (int i = 1; i < 4; ++i)
		{
			CDB_GT_Model_Tile_Selector t;

			//			"_S001_T001_";

			std::stringstream dsbuf;
			dsbuf << "_D101_S" << std::setfill('0')
				<< std::setw(3) << abs(i) << "_T" << std::setfill('0')
				<< std::setw(3) << abs(Tnum) << "_";
			datasetstr = dsbuf.str();

			std::stringstream f1buf;
			f1buf << cdbRootDir
				<< "\\Tiles"
				<< "\\" << m_lat_str
				<< "\\" << m_lon_str
				<< "\\" << m_LayerName
				<< "\\" << m_lod_str
				<< "\\" << m_uref_str
				<< "\\" << m_lat_str << m_lon_str << datasetstr << m_lod_str
				<< "_" << m_uref_str << "_" << m_rref_str << filetype;
			t.TilePrimaryShapeName = f1buf.str();

			std::stringstream laybuf;
			laybuf << "101_GTFeature_S" << std::setfill('0') << std::setw(3) << abs(i) << "_T"
				<< std::setfill('0') << std::setw(3) << abs(Tnum) << "_Pnt";
			t.PrimaryLayerName = laybuf.str();

			std::stringstream ds2buf;
			ds2buf << "_D101_S" << std::setfill('0')
				<< std::setw(3) << abs(i) << "_T" << std::setfill('0')
				<< std::setw(3) << abs(Tnum + 1) << "_";
			datasetstr = ds2buf.str();

			std::stringstream f2buf;
			f2buf << cdbRootDir
				<< "\\Tiles"
				<< "\\" << m_lat_str
				<< "\\" << m_lon_str
				<< "\\" << m_LayerName
				<< "\\" << m_lod_str
				<< "\\" << m_uref_str
				<< "\\" << m_lat_str << m_lon_str << datasetstr << m_lod_str
				<< "_" << m_uref_str << "_" << m_rref_str << filetype2str;
			t.TileSecondaryShapeName = f2buf.str();

			std::stringstream clslaybuf;
			clslaybuf << "101_GTFeature_S" << std::setfill('0') << std::setw(3) << abs(i) << "_T"
				<< std::setfill('0') << std::setw(3) << abs(Tnum + 1) << "_Cls";
			t.ClassLayerName = clslaybuf.str();

			m_GTModelSet.push_back(t);
		}
	}
	if ((m_TileType == Elevation) || (m_TileType == ElevationCache))
	{
		m_SubordinateName = subordinatebuf.str();
	}


	if (m_TileType == GeoTypicalModel)
	{
		m_FileExists = false;
		for (size_t id = 0; id < m_GTModelSet.size(); ++id)
		{
			m_GTModelSet[id].PrimaryExists = validate_tile_name(m_GTModelSet[id].TilePrimaryShapeName);
			if(m_GTModelSet[id].PrimaryExists)
				m_FileExists = true;

			m_GTModelSet[id].ClassExists = validate_tile_name(m_GTModelSet[id].TileSecondaryShapeName);
		}
	}
	else if (m_TileType == GeoSpecificModel)
	{
		m_FileExists = validate_tile_name(m_FileName);
		m_ModelSet.ModelDbfNameExists = validate_tile_name(m_ModelSet.ModelDbfName);
		m_ModelSet.ModelGeometryNameExists = validate_tile_name(m_ModelSet.ModelGeometryName);
		m_ModelSet.ModelTextureNameExists = validate_tile_name(m_ModelSet.ModelTextureName);

	}
	else
	{
		m_FileExists = validate_tile_name(m_FileName);
		if ((m_TileType == Elevation) || (m_TileType == ElevationCache))
		{
			if (s_EnableBathymetry)
				m_Subordinate_Exists = validate_tile_name(m_SubordinateName);
			else
				m_Subordinate_Exists = false;
		}
	}

	if (((m_TileType == GeoPackageMap)) && (!m_FileExists))
	{
		size_t spos = m_FileName.find_last_of(".gpkg");
		std::string xmlname = m_FileName;
		if (spos != std::string::npos)
		{
			xmlname = Xml_Name(m_FileName);
		}
		if (validate_tile_name(xmlname))
		{
			m_FileName = xmlname;
			m_FileExists = true;
		}
	}

	m_Pixels.degPerPix.Xpos = (m_TileExtent.East - m_TileExtent.West) / (double)(m_Pixels.pixX);
	m_Pixels.degPerPix.Ypos = (m_TileExtent.North - m_TileExtent.South) / (double)(m_Pixels.pixY);


}


CDB_Tile::~CDB_Tile()
{
	Close_Dataset();

	Free_Buffers();
}

std::string CDB_Tile::FileName(int sel)
{
	if (sel <= 0)
		return m_FileName;
	else if (m_TileType == GeoTypicalModel)
	{
		return m_GTModelSet[sel].TilePrimaryShapeName;
	}
	else
		return m_FileName;
}

int CDB_Tile::CDB_LOD_Num(void)
{
	return m_CDB_LOD_Num;
}

void CDB_Tile::Free_Resources(void)
{
	Close_Dataset();
	Free_Buffers();
}

int CDB_Tile::GetPathComponents(std::string& lat_str, std::string& lon_str, std::string& lod_str,
								std::string& uref_str, std::string& rref_str)
{

	int cdbLod = 0;

	//Determine the CDB LOD
	double keylonspace = m_TileExtent.East - m_TileExtent.West;
	double keylatspace = m_TileExtent.North - m_TileExtent.South;

	double tilesperdeg = 1.0 / keylatspace;
	double tilesperdegX = 1.0 / keylonspace;

	if (tilesperdeg < 0.99)
	{
		//This is a multi-tile cash tile
		double lnum = 1.0 / tilesperdeg;
		int itiles = (int)(round(lnum / 2.0));
		cdbLod = -1;
		while (itiles > 1)
		{
			itiles /= 2;
			--cdbLod;
		}
	}
	else
	{
		if (m_Pixels.pixX < 1024)
		{
			// In this case the lod num has already been passed into the class
			// just use it.
			cdbLod = m_CDB_LOD_Num;
		}
		else
		{
			cdbLod = 0;
			int itiles = (int)round(tilesperdeg);
			while (itiles > 1)
			{
				itiles /= 2;
				++cdbLod;
			}

		}
	}
	int tile_x, tile_y;

	double Base_lon = m_TileExtent.West;

	if (cdbLod > 0)
	{
		double Base_lat = (double)((int)m_TileExtent.South);
		if (m_TileExtent.South < Base_lat)
			Base_lat -= 1.0;
		double off = m_TileExtent.South - Base_lat;
		tile_y = (int)round(off * tilesperdeg);

		double lon_step = Get_Lon_Step(m_TileExtent.South);
		Base_lon = (double)((int)m_TileExtent.West);

		if (lon_step != 1.0)
		{
			int checklon = (int)abs(Base_lon);
			if (checklon % (int)lon_step)
			{
				double sign = Base_lon < 0.0 ? -1.0 : 1.0;
				checklon = (checklon / (int)lon_step) * (int)lon_step;
				Base_lon = (double)checklon * sign;
				if (sign < 0.0)
					Base_lon -= lon_step;
			}
		}

		if (m_TileExtent.West < Base_lon)
			Base_lon -= lon_step;

		off = m_TileExtent.West - Base_lon;
		tile_x = (int)round(off * tilesperdegX);
	}
	else
	{
		tile_x = tile_y = 0;
	}
	//Determine the base lat lon directory
	double lont = (double)((int)Base_lon);
	//make sure there wasn't a rounding error
	if (abs((lont + 1.0) - Base_lon) < DBL_EPSILON)
		lont += 1.0;
	else if (Base_lon < lont)//Where in the Western Hemisphere round down.
		lont -= 1.0;


	int londir = (int)lont;
	std::stringstream format_stream_1;
	format_stream_1 << ((londir < 0) ? "W" : "E") << std::setfill('0')
		<< std::setw(3) << abs(londir);
	lon_str = format_stream_1.str();


	double latt = (double)((int)m_TileExtent.South);
	//make sure there wasn't a rounding error
	if (abs((latt + 1.0) - m_TileExtent.South) < DBL_EPSILON)
		latt += 1.0;
	else if (m_TileExtent.South < latt) //Where in the Southern Hemisphere round down.
		latt -= 1.0;

	int latdir = (int)latt;
	std::stringstream format_stream_2;
	format_stream_2 << ((latdir < 0) ? "S" : "N") << std::setfill('0')
		<< std::setw(2) << abs(latdir);
	lat_str = format_stream_2.str();

	// Set the LOD of the request
	std::stringstream lod_stream;
	if (cdbLod < 0)
		lod_stream << "LC" << std::setfill('0') << std::setw(2) << abs(cdbLod);
	else
		lod_stream << "L" << std::setfill('0') << std::setw(2) << cdbLod;
	lod_str = lod_stream.str();

	if (cdbLod < 1)
	{
		//There is only one tile in cdb levels 0 and below
		//
		uref_str = "U0";
		rref_str = "R0";
	}
	else
	{
		// Determine UREF
		std::stringstream uref_stream;
		uref_stream << "U" << tile_y;
		uref_str = uref_stream.str();

		// Determine RREF
		std::stringstream rref_stream;
		rref_stream << "R" << tile_x;
		rref_str = rref_stream.str();
	}
	return cdbLod;
}


void CDB_Tile::Allocate_Buffers(void)
{
	int bandbuffersize = m_Pixels.pixX * m_Pixels.pixY;
	if ((m_TileType == Imagery) || (m_TileType == ImageryCache))
	{
		if (!m_GDAL.reddata)
		{
			m_GDAL.reddata = new unsigned char[bandbuffersize * 3];
			m_GDAL.greendata = m_GDAL.reddata + bandbuffersize;
			m_GDAL.bluedata = m_GDAL.greendata + bandbuffersize;
		}
	}
	else if ((m_TileType == Elevation) || (m_TileType == ElevationCache))
	{
		if (!m_GDAL.elevationdata)
			m_GDAL.elevationdata = new float[bandbuffersize];
		if (m_Subordinate_Exists || m_Subordinate_Tile)
		{
			if (!m_GDAL.subord_elevationdata)
				m_GDAL.subord_elevationdata = new float[bandbuffersize];
		}
	}
}

void CDB_Tile::Free_Buffers(void)
{
	if (m_GDAL.reddata)
	{
		delete m_GDAL.reddata;
		m_GDAL.reddata = NULL;
		m_GDAL.greendata = NULL;
		m_GDAL.bluedata = NULL;
	}
	if (m_GDAL.elevationdata)
	{
		delete m_GDAL.elevationdata;
		m_GDAL.elevationdata = NULL;
	}
	if (m_GDAL.subord_elevationdata)
	{
		delete m_GDAL.subord_elevationdata;
		m_GDAL.subord_elevationdata = NULL;
	}
	if (m_Tile_Status == Loaded)
	{
		if (m_GDAL.poDataset)
			m_Tile_Status = Opened;
		else
			m_Tile_Status = Created;
	}
}

void CDB_Tile::Close_Dataset(void)
{

	if (m_GDAL.poDataset)
	{
		GDALClose(m_GDAL.poDataset);
		m_GDAL.poDataset = NULL;
	}
	if (m_GDAL.soDataset)
	{
		GDALClose(m_GDAL.soDataset);
		m_GDAL.soDataset = NULL;
	}
	if (m_TileType == GeoTypicalModel)
		Close_GT_Model_Tile();
	else if (m_TileType == GeoSpecificModel)
		Close_GS_Model_Tile();

	m_Tile_Status = Created;

}

bool CDB_Tile::Open_Tile(void)
{
	if (m_GDAL.poDataset)
		return true;

	GDALOpenInfo oOpenInfo(m_FileName.c_str(), GA_ReadOnly);
	if (m_TileType == Imagery)
	{
		m_GDAL.poDriver = Gbl_TileDrivers.cdb_JP2Driver;
	}
	else if (m_TileType == ImageryCache)
	{
		m_GDAL.poDriver = Gbl_TileDrivers.cdb_GTIFFDriver;
	}
	else if (m_TileType == Elevation)
	{
		m_GDAL.poDriver = Gbl_TileDrivers.cdb_GTIFFDriver;
	}
	else if (m_TileType == ElevationCache)
	{
		m_GDAL.poDriver = Gbl_TileDrivers.cdb_HFADriver;
	}
	else if (m_TileType == GeoTypicalModel)
	{
		m_GDAL.poDriver = Gbl_TileDrivers.cdb_ShapefileDriver;
		return Open_GT_Model_Tile();
	}
	else if (m_TileType == GeoSpecificModel)
	{
		m_GDAL.poDriver = Gbl_TileDrivers.cdb_ShapefileDriver;
		return Open_GS_Model_Tile();
	}
	else if (m_TileType == GeoPackageMap)
	{
		m_GDAL.poDriver = Gbl_TileDrivers.cdb_GeoPackageDriver;
		return Open_GP_Map_Tile();
	}
	m_GDAL.poDataset = (GDALDataset *)m_GDAL.poDriver->pfnOpen(&oOpenInfo);
	if (m_Subordinate_Exists)
	{
		GDALOpenInfo sOpenInfo(m_SubordinateName.c_str(), GA_ReadOnly);
		m_GDAL.soDataset = (GDALDataset *)m_GDAL.poDriver->pfnOpen(&sOpenInfo);
	}
	if (!m_GDAL.poDataset)
	{
		return false;
	}
	if (m_Subordinate_Exists && !m_GDAL.soDataset)
	{
		return false;
	}
	m_GDAL.poDataset->GetGeoTransform(m_GDAL.adfGeoTransform);
	m_Tile_Status = Opened;
	return true;
}

bool CDB_Tile::Open_GS_Model_Tile(void)
{
	if (m_FileExists && m_ModelSet.ModelDbfNameExists && m_ModelSet.ModelGeometryNameExists)
	{
		GDALOpenInfo oOpenInfoP(m_FileName.c_str(), GA_ReadOnly);
		m_ModelSet.PrimaryTileOgr = m_GDAL.poDriver->pfnOpen(&oOpenInfoP);
		if (!m_ModelSet.PrimaryTileOgr)
			return false;
		GDALOpenInfo oOpenInfoC(m_ModelSet.ModelDbfName.c_str(), GA_ReadOnly);
		m_ModelSet.ClassTileOgr = m_GDAL.poDriver->pfnOpen(&oOpenInfoC);
		if (!m_ModelSet.ClassTileOgr)
		{
			//Check for junk files clogging up the works
			std::string shx = Set_FileType(m_ModelSet.ModelDbfName, ".shx");
			if (validate_tile_name(shx))
			{
				if (::DeleteFile(shx.c_str()) == 0)
				{
					return false;
				}
			}
			std::string shp = Set_FileType(m_ModelSet.ModelDbfName, ".shp");
			if (validate_tile_name(shp))
			{
				if (::DeleteFile(shp.c_str()) == 0)
				{
					return false;
				}
			}
			m_ModelSet.ClassTileOgr = m_GDAL.poDriver->pfnOpen(&oOpenInfoC);
			if (!m_ModelSet.ClassTileOgr)
				return false;
		}

	}
	else
		return false;
	m_Tile_Status = Opened;
	return true;
}

bool CDB_Tile::Open_GT_Model_Tile(void)
{
	bool have_an_opening = false;
	for (size_t i = 0; i < m_GTModelSet.size(); ++i)
	{
		if (m_GTModelSet[i].PrimaryExists && m_GTModelSet[i].ClassExists)
		{
			GDALOpenInfo oOpenInfoP(m_GTModelSet[i].TilePrimaryShapeName.c_str(), GA_ReadOnly);
			m_GTModelSet[i].PrimaryTileOgr = m_GDAL.poDriver->pfnOpen(&oOpenInfoP);
			if (!m_GTModelSet[i].PrimaryTileOgr)
				continue;
			GDALOpenInfo oOpenInfoC(m_GTModelSet[i].TileSecondaryShapeName.c_str(), GA_ReadOnly);
			m_GTModelSet[i].ClassTileOgr = m_GDAL.poDriver->pfnOpen(&oOpenInfoC);
			if (!m_GTModelSet[i].ClassTileOgr)
			{
				//Check for junk files clogging up the works
				std::string shx = Set_FileType(m_GTModelSet[i].TileSecondaryShapeName, ".shx");
				if (validate_tile_name(shx))
				{
					if (::DeleteFile(shx.c_str()) == 0)
					{
						continue;
					}
				}
				std::string shp = Set_FileType(m_GTModelSet[i].TileSecondaryShapeName, ".shp");
				if (validate_tile_name(shp))
				{
					if (::DeleteFile(shp.c_str()) == 0)
					{
						continue;
					}
				}
				m_GTModelSet[i].ClassTileOgr = m_GDAL.poDriver->pfnOpen(&oOpenInfoC);
				if (!m_GTModelSet[i].ClassTileOgr)
					continue;
			}
			if (m_GTModelSet[i].PrimaryTileOgr && m_GTModelSet[i].ClassTileOgr)
				have_an_opening = true;
		}
	}
	if(have_an_opening)
		m_Tile_Status = Opened;
	return have_an_opening;
}

bool CDB_Tile::Open_GP_Map_Tile(void)
{
	if (m_FileExists && (m_FileName.find(".gpkg") != std::string::npos))
	{
//		GDALOpenInfo oOpenInfoP(m_FileName.c_str(), GA_ReadOnly | GDAL_OF_VECTOR);
//		m_GDAL.poDataset = m_GDAL.poDriver->pfnOpen(&oOpenInfoP);
		char * drivers[2];
		drivers[0] = "GPKG";
		drivers[1] = NULL;
		m_GDAL.poDataset = (GDALDataset *)GDALOpenEx(m_FileName.c_str(), GDAL_OF_VECTOR | GA_ReadOnly | GDAL_OF_SHARED, drivers, NULL, NULL);
		if (!m_GDAL.poDataset)
			return false;
	}
	else
		return false;
	m_Tile_Status = Opened;
	return true;
}

OGRLayer * CDB_Tile::Map_Tile_Layer(std::string LayerName)
{
	if (m_TileType != GeoPackageMap)
		return NULL;
	if (!m_GDAL.poDataset)
		return NULL;
	return m_GDAL.poDataset->GetLayerByName(LayerName.c_str());
}

GDALDataset * CDB_Tile::Map_Tile_Dataset(void)
{
	if (m_TileType != GeoPackageMap)
		return NULL;
	return m_GDAL.poDataset;
}
void CDB_Tile::Close_GT_Model_Tile(void)
{
	for (size_t i = 0; i < m_GTModelSet.size(); ++i)
	{
		if (m_GTModelSet[i].PrimaryTileOgr)
		{
			GDALClose(m_GTModelSet[i].PrimaryTileOgr);
			m_GTModelSet[i].PrimaryTileOgr = NULL;
			m_GTModelSet[i].PrimaryLayer = NULL;
		}
		if (m_GTModelSet[i].ClassTileOgr)
		{
			GDALClose(m_GTModelSet[i].ClassTileOgr);
			m_GTModelSet[i].ClassTileOgr = NULL;
		}
		m_GTModelSet[i].clsMap.clear();
	}
}

void CDB_Tile::Close_GS_Model_Tile(void)
{
	if (m_ModelSet.PrimaryTileOgr)
	{
		GDALClose(m_ModelSet.PrimaryTileOgr);
		m_ModelSet.PrimaryTileOgr = NULL;
		m_ModelSet.PrimaryLayer = NULL;
	}

	if (m_ModelSet.ClassTileOgr)
	{
		GDALClose(m_ModelSet.ClassTileOgr);
		m_ModelSet.ClassTileOgr = NULL;
	}

	m_ModelSet.clsMap.clear();
	m_ModelSet.archiveFileList.clear();
}


bool CDB_Tile::Init_Model_Tile(int sel)
{
	if (m_Tile_Status != Opened)
	{
		if (!Open_Tile())
			return false;
	}

	if (m_TileType == GeoSpecificModel)
	{
		return Init_GS_Model_Tile();
	}
	else if (m_TileType == GeoTypicalModel)
	{
		return Init_GT_Model_Tile(sel);
	}
	return false;
}

bool CDB_Tile::Init_Map_Tile(void)
{
	if (m_Tile_Status != Opened)
	{
		if (!Open_Tile())
			return false;
	}
	return true;
}

bool CDB_Tile::Model_Geometry_Name(std::string &GeometryName)
{
	if (m_TileType != GeoSpecificModel)
	{
		GeometryName = "";
		return false;
	}
	GeometryName = m_ModelSet.ModelGeometryName;
	return m_ModelSet.ModelGeometryNameExists;
}

bool CDB_Tile::Model_Texture_Directory(std::string &TextureDir)
{
	if (m_TileType != GeoSpecificModel)
	{
		TextureDir = "";
		return false;
	}
	TextureDir = Model_TextureDir();
	return validate_tile_name(TextureDir);
}

bool CDB_Tile::Model_Texture_Archive(std::string &TextureArchive)
{
	if (m_TileType != GeoSpecificModel)
	{
		TextureArchive = "";
		return false;
	}
	TextureArchive = m_ModelSet.ModelTextureName;
	return m_ModelSet.ModelTextureNameExists;
}

OGRFeature * CDB_Tile::Next_Valid_Feature(int sel, bool inflated, std::string &ModelKeyName, std::string &FullModelName, std::string &ArchiveFileName,
										  bool &Model_in_Archive)
{
	if (m_TileType == GeoSpecificModel)
		return Next_Valid_Geospecific_Feature(inflated, ModelKeyName, FullModelName, ArchiveFileName, Model_in_Archive);
	else
		return Next_Valid_GeoTypical_Feature(sel, ModelKeyName, FullModelName, Model_in_Archive);
}

OGRFeature * CDB_Tile::Next_Valid_Geospecific_Feature(bool inflated, std::string &ModelKeyName, std::string &FullModelName, std::string &ArchiveFileName,
													   bool &Model_in_Archive)
{
	bool valid = false;
	bool done = false;
	OGRFeature *f = NULL;

	while (!valid && !done)
	{
		f = m_ModelSet.PrimaryLayer->GetNextFeature();
		if (!f)
		{
			done = true;
			break;
		}
		valid = true;
		std::string cnam = f->GetFieldAsString("CNAM");
		if (!cnam.empty())
		{
			CDB_Model_RuntimeMap::iterator mi = m_ModelSet.clsMap.find(cnam);
			if (mi == m_ModelSet.clsMap.end())
				valid = false;
			else
			{
				CDB_Model_Runtime_Class myExtents = m_ModelSet.clsMap[cnam];
				ModelKeyName = Model_KeyName(myExtents.FACC_value, myExtents.FSC_value, myExtents.Model_Base_Name);
				if (inflated)
				{
					FullModelName = Model_FullFileName(myExtents.FACC_value, myExtents.FSC_value, myExtents.Model_Base_Name);
					Model_in_Archive = validate_tile_name(FullModelName);
				}
				else
				{
					FullModelName = Model_FileName(myExtents.FACC_value, myExtents.FSC_value, myExtents.Model_Base_Name);
					ArchiveFileName = archive_validate_modelname(m_ModelSet.archiveFileList, FullModelName);
					if (ArchiveFileName.empty())
						Model_in_Archive = false;
					else
						Model_in_Archive = true;
				}
			}
		}
		else
			valid = false;
	}
	return f;
}

OGRFeature * CDB_Tile::Next_Valid_GeoTypical_Feature(int sel, std::string &ModelKeyName, std::string &ModelFullName, bool &Model_in_Archive)
{
	bool valid = false;
	bool done = false;
	OGRFeature *f = NULL;

	while (!valid && !done)
	{
		f = m_GTModelSet[sel].PrimaryLayer->GetNextFeature();
		if (!f)
		{
			done = true;
			break;
		}
		valid = true;
		std::string cnam = f->GetFieldAsString("CNAM");
		if (!cnam.empty())
		{
			CDB_Model_RuntimeMap::iterator mi = m_GTModelSet[sel].clsMap.find(cnam);
			if (mi == m_GTModelSet[sel].clsMap.end())
				valid = false;
			else
			{
				CDB_Model_Runtime_Class myExtents = m_GTModelSet[sel].clsMap[cnam];
				ModelKeyName = Model_KeyName(myExtents.FACC_value, myExtents.FSC_value, myExtents.Model_Base_Name);
				ModelFullName = GeoTypical_FullFileName(ModelKeyName);
				Model_in_Archive = validate_tile_name(ModelFullName);
			}
		}
		else
			valid = false;
	}
	return f;
}

std::string CDB_Tile::Model_KeyName(std::string &FACC_value, std::string &FSC_Value, std::string &BaseFileName)
{
	std::stringstream modbuf;
	modbuf << FACC_value << "_" << FSC_Value << "_"
		<< BaseFileName << ".flt";
	return modbuf.str();
}

bool CDB_Tile::Init_GS_Model_Tile(void)
{
	if (!m_ModelSet.ClassTileOgr)
		return false;
	m_ModelSet.PrimaryLayer = m_ModelSet.PrimaryTileOgr->GetLayer(0);
	if (!m_ModelSet.PrimaryLayer)
		return false;
	m_ModelSet.PrimaryLayer->ResetReading();
	OGRLayer *poLayer = m_ModelSet.ClassTileOgr->GetLayer(0);
	bool have_class = Load_Class_Map(poLayer, m_ModelSet.clsMap);
	bool have_archive = Load_Archive(m_ModelSet.ModelGeometryName, m_ModelSet.archiveFileList);
	return (have_class && have_archive);
}

bool CDB_Tile::Load_Archive(std::string ArchiveName, osgDB::Archive::FileNameList &archiveFileList)
{
	osg::ref_ptr<osgDB::Archive> ar = NULL;
	ar = osgDB::openArchive(ArchiveName, osgDB::ReaderWriter::ArchiveStatus::READ);
	if (ar)
	{
		ar->getFileNames(archiveFileList);
		ar.release();
		return true;
	}
	else
		return false;
}

std::string CDB_Tile::archive_validate_modelname(osgDB::Archive::FileNameList &archiveFileList, std::string &filename)
{
	std::string result = "";
	for (osgDB::Archive::FileNameList::const_iterator f = archiveFileList.begin(); f != archiveFileList.end(); ++f)
	{
		const std::string comp = *f;
		if (comp.find(filename) != std::string::npos)
		{
			return comp;
		}
	}
	return result;
}

bool CDB_Tile::Load_Class_Map(OGRLayer * poLayer, CDB_Model_RuntimeMap &clsMap)
{
	OGRFeatureDefn * poFDefn = poLayer->GetLayerDefn();
	int name_attr_index = Find_Field_Index(poFDefn, "MODL", OFTString);
	if (name_attr_index < 0)
	{
		return false;
	}

	int cnam_attr_index = Find_Field_Index(poFDefn, "CNAM", OFTString);
	if (cnam_attr_index < 0)
	{
		return false;
	}

	int facc_index = Find_Field_Index(poFDefn, "FACC", OFTString);
	if (facc_index < 0)
	{
		return false;
	}

	int fsc_index = Find_Field_Index(poFDefn, "FSC", OFTInteger);
	if (fsc_index < 0)
	{
		return false;
	}

	poLayer->ResetReading();
	OGRFeature* dbf_feature;
	while ((dbf_feature = poLayer->GetNextFeature()) != NULL)
	{
		CDB_Model_Runtime_Class nextEntry;
		std::string Key = nextEntry.set_class(dbf_feature, cnam_attr_index, name_attr_index, facc_index, fsc_index);
		clsMap.insert(std::pair<std::string, CDB_Model_Runtime_Class>(Key, nextEntry));
		OGRFeature::DestroyFeature(dbf_feature);
	}
	return true;
}

int CDB_Tile::Find_Field_Index(OGRFeatureDefn *poFDefn, std::string fieldname, OGRFieldType Type)
{
	int dbfieldcnt = poFDefn->GetFieldCount();
	for (int dbffieldIdx = 0; dbffieldIdx < dbfieldcnt; ++dbffieldIdx)
	{
		OGRFieldDefn *po_FieldDefn = poFDefn->GetFieldDefn(dbffieldIdx);
		std::string thisname = po_FieldDefn->GetNameRef();
		if (thisname.compare(fieldname) == 0)
		{
			if (po_FieldDefn->GetType() == Type)
				return dbffieldIdx;
		}
	}
	return -1;
}

bool CDB_Tile::Init_GT_Model_Tile(int sel)
{
	if (!m_GTModelSet[sel].ClassTileOgr)
		return false;
	m_GTModelSet[sel].PrimaryLayer = m_GTModelSet[sel].	PrimaryTileOgr->GetLayer(0);
	if (!m_GTModelSet[sel].PrimaryLayer)
		return false;
	m_GTModelSet[sel].PrimaryLayer->ResetReading();
	OGRLayer *poLayer = m_GTModelSet[sel].ClassTileOgr->GetLayer(0);
	return Load_Class_Map(poLayer, m_GTModelSet[sel].clsMap);
}

int CDB_Tile::Model_Sel_Count(void)
{
	if (m_TileType == GeoSpecificModel)
		return 1;
	else if (m_TileType == GeoTypicalModel)
		return (int)m_GTModelSet.size();
	else
		return -1;
}

bool CDB_Tile::Read(void)
{
	if (!m_GDAL.poDataset)
		return false;

	if (m_Tile_Status == Loaded)
		return true;


	if ((m_TileType == Imagery) || (m_TileType == ImageryCache))
	{
		CPLErr gdal_err = m_GDAL.poDataset->RasterIO(GF_Read, 0, 0, m_Pixels.pixX, m_Pixels.pixY,
													 m_GDAL.reddata, m_Pixels.pixX, m_Pixels.pixY, GDT_Byte, 3, NULL, 0, 0, 0);
		if (gdal_err == CE_Failure)
		{
			return false;
		}

	}
	else if ((m_TileType == Elevation) || (m_TileType == ElevationCache))
	{
		GDALRasterBand * ElevationBand = m_GDAL.poDataset->GetRasterBand(1);

		CPLErr gdal_err = ElevationBand->RasterIO(GF_Read, 0, 0, m_Pixels.pixX, m_Pixels.pixY,
			                                      m_GDAL.elevationdata, m_Pixels.pixX, m_Pixels.pixY, GDT_Float32, 0, 0);
		if (gdal_err == CE_Failure)
		{
			return false;
		}
		if (m_Subordinate_Exists)
		{
			if (!m_GDAL.soDataset)
				return false;

			GDALRasterBand * SubordElevationBand = m_GDAL.soDataset->GetRasterBand(1);

			gdal_err = SubordElevationBand->RasterIO(GF_Read, 0, 0, m_Pixels.pixX, m_Pixels.pixY,
													 m_GDAL.subord_elevationdata, m_Pixels.pixX, m_Pixels.pixY, GDT_Float32, 0, 0);
			if (gdal_err == CE_Failure)
			{
				return false;
			}
		}
	}
	m_Tile_Status = Loaded;

	return true;
}

void CDB_Tile::Fill_Tile(void)
{
	int buffsz = m_Pixels.pixX * m_Pixels.pixY;
	if (m_TileType == Imagery)
	{
		for (int i = 0; i < buffsz; ++i)
		{
			m_GDAL.reddata[i] = 127;
			m_GDAL.greendata[i] = 127;
			m_GDAL.bluedata[i] = 127;
		}
	}
	else if ((m_TileType == Elevation) || (m_TileType == ElevationCache))
	{
		for (int i = 0; i < buffsz; ++i)
		{
			m_GDAL.elevationdata[i] = 0.0f;
		}
		if (m_Subordinate_Tile)
		{
			for (int i = 0; i < buffsz; ++i)
			{
				m_GDAL.subord_elevationdata[i] = 0.0f;
			}
		}

	}
	else if (m_TileType == ImageryCache)
	{
		for (int i = 0; i < buffsz; ++i)
		{
			m_GDAL.reddata[i] = 0;
			m_GDAL.greendata[i] = 0;
			m_GDAL.bluedata[i] = 0;
		}
	}
}

bool CDB_Tile::Tile_Exists(int sel)
{
	if(sel < 0)
		return m_FileExists;
	else if (m_TileType == GeoSpecificModel)
	{
		if (m_FileExists && m_ModelSet.ModelDbfNameExists && m_ModelSet.ModelGeometryNameExists)
			return true;
		else
			return false;
	}
	else if (m_TileType == GeoTypicalModel)
	{
		if (m_GTModelSet[sel].PrimaryExists && m_GTModelSet[sel].ClassExists)
			return true;
		else
			return false;
	}
	return m_FileExists;
}

bool CDB_Tile::Subordinate_Exists(void)
{
	return m_Subordinate_Exists;
}

void CDB_Tile::Set_Subordinate(bool value)
{
	m_Subordinate_Tile = value;
}

bool CDB_Tile::Build_Cache_Tile(bool save_cache)
{
	//This is not actually part of the CDB specification but
	//necessary to support an osgEarth global profile
	//Build a list of the tiles to use for this cache tile

	double MinLat = m_TileExtent.South;
	double MinLon = m_TileExtent.West;
	double MaxLat = m_TileExtent.North;
	double MaxLon = m_TileExtent.East;

	CDB_Tile_Type subTileType;
	if (m_TileType == ImageryCache)
	{
		subTileType = Imagery;
	}
	else if (m_TileType == ElevationCache)
	{
		subTileType = Elevation;
	}
	CDB_TilePV Tiles;

	bool done = false;
	CDB_Tile_Extent thisTileExtent;
	thisTileExtent.West = MinLon;
	thisTileExtent.South = MinLat;
	double lonstep = Get_Lon_Step(thisTileExtent.South);
	double sign;
	while (!done)
	{
		if (lonstep != 1.0)
		{
			int checklon = (int)abs(thisTileExtent.West);
			if (checklon % (int)lonstep)
			{
				sign = thisTileExtent.West < 0.0 ? -1.0 : 1.0;
				checklon = (checklon / (int)lonstep) * (int)lonstep;
				thisTileExtent.West = (double)checklon * sign;
				if (sign < 0.0)
					thisTileExtent.West -= lonstep;
			}
		}
		thisTileExtent.East = thisTileExtent.West + lonstep;
		thisTileExtent.North = thisTileExtent.South + 1.0;

		CDB_TileP LodTile = new CDB_Tile(m_cdbRootDir, m_cdbCacheDir, subTileType, m_DataSet, &thisTileExtent, m_CDB_LOD_Num);

		if (LodTile->Tile_Exists())
		{
			Tiles.push_back(LodTile);
		}
		else
			delete LodTile;

		thisTileExtent.West += lonstep;
		if (thisTileExtent.West > MaxLon)
		{
			thisTileExtent.West = MinLon;
			thisTileExtent.South += 1.0;
			lonstep = Get_Lon_Step(thisTileExtent.South);
			if (thisTileExtent.South > MaxLat)
				done = true;
		}
	}

	int tilecnt = (int)Tiles.size();
	if (tilecnt <= 0)
		return false;

	Build_From_Tiles(&Tiles);

	if (save_cache && (m_Tile_Status == Loaded))
	{
		Save();
		m_FileExists = true;
	}

	//Clean up
	for each (CDB_TileP Tile in Tiles)
	{
		if (Tile)
		{
			delete Tile;
			Tile = NULL;
		}
	}
	Tiles.clear();

	return true;
}

bool CDB_Tile::Build_Earth_Tile(void)
{
	//Build an Earth Profile tile for Latitudes above and below 50 deg

	double MinLon = m_TileExtent.West;

	CDB_Tile_Type subTileType = m_TileType;

	CDB_TilePV Tiles;

	bool done = false;

	OE_DEBUG "Build_Earth_Tile with " << m_FileName.c_str() << std::endl;
	OE_DEBUG "Build_Earth_Tile my extent " << m_TileExtent.North << " " <<m_TileExtent.South << " " << m_TileExtent.East << " " << m_TileExtent.West << std::endl;

	double lonstep = Get_Lon_Step(m_TileExtent.South);
	lonstep /= Gbl_CDB_Tiles_Per_LOD[m_CDB_LOD_Num];

	double incrs = round(MinLon / lonstep);
	double test = incrs * lonstep;
	if (test != MinLon)
	{
		MinLon = test;
	}

	CDB_Tile_Extent thisTileExtent;
	thisTileExtent.West = MinLon;
	thisTileExtent.South = m_TileExtent.South;
	thisTileExtent.East = thisTileExtent.West + lonstep;
	thisTileExtent.North = m_TileExtent.North;

	OE_DEBUG "Build_Earth_Tile cdb extent " << thisTileExtent.North << " " << thisTileExtent.South << " " << thisTileExtent.East << " " << thisTileExtent.West << std::endl;

	//Now get the actual cdb tile with the correct CDB extents
	CDB_TileP LodTile = new CDB_Tile(m_cdbRootDir, m_cdbCacheDir, subTileType, m_DataSet, &thisTileExtent);

	OE_DEBUG "Build_Earth_Tile cdb tile " << LodTile->FileName().c_str() << std::endl;

	if (LodTile->Tile_Exists())
	{
		if (LodTile->Subordinate_Exists())
			m_Subordinate_Tile = true;
		OE_DEBUG "CDB_Tile found " << LodTile->FileName().c_str() << std::endl;
		Tiles.push_back(LodTile);
	}
	else
		delete LodTile;

	int tilecnt = (int)Tiles.size();
	//if the tile does not exist
	//there is no tile to build
	if (tilecnt <= 0)
		return false;

	//Build the osgearth tile from the cdb tile
	Build_From_Tiles(&Tiles, true);

	//clean up
	for each (CDB_TileP Tile in Tiles)
	{
		if (Tile)
		{
			delete Tile;
			Tile = NULL;
		}
	}
	Tiles.clear();

	return true;
}

double CDB_Tile::Get_Lon_Step(double Latitude)
{
	double test = abs(Latitude);
	double step;
	if (Latitude >= 0.0)
	{
		if (test < 50.0)
			step = 1.0;
		else if (test >= 50.0 && test < 70.0)
			step = 2.0;
		else if (test >= 70.0 && test < 75.0)
			step = 3.0;
		else if (test >= 75.0 && test < 80.0)
			step = 4.0;
		else if (test >= 80.0 && test < 89.0)
			step = 6.0;
		else
			step = 12.0;
	}
	else
	{
		if (test <= 50.0)
			step = 1.0;
		else if (test > 50.0 && test <= 70.0)
			step = 2.0;
		else if (test > 70.0 && test <= 75.0)
			step = 3.0;
		else if (test > 75.0 && test <= 80.0)
			step = 4.0;
		else if (test > 80.0 && test <= 89.0)
			step = 6.0;
		else
			step = 12.0;

	}
	return step;
}

void CDB_Tile::Disable_Bathyemtry(bool value)
{
	if (value)
		s_EnableBathymetry = false;
	else
		s_EnableBathymetry = true;
}

bool CDB_Tile::Initialize_Tile_Drivers(std::string &ErrorMsg)
{
	ErrorMsg = "";
	if (Gbl_TileDrivers.cdb_drivers_initialized)
		return true;

	std::string	cdb_JP2DriverNames[JP2DRIVERCNT];
	//The JP2 Driver Names should be ordered based on read performance however this has not been done yet.
	cdb_JP2DriverNames[0] = "JP2ECW";		//ERDAS supplied JP2 Plugin
	cdb_JP2DriverNames[1] = "JP2OpenJPEG";  //LibOpenJPEG2000
	cdb_JP2DriverNames[2] = "JPEG2000";	    //JASPER
	cdb_JP2DriverNames[3] = "JP2KAK";		//Kakadu Library
	cdb_JP2DriverNames[4] = "JP2MrSID";	    //MR SID SDK

	//Find a jpeg2000 driver for the image layer.
	int dcount = 0;
	while ((Gbl_TileDrivers.cdb_JP2Driver == NULL) && (dcount < JP2DRIVERCNT))
	{
		Gbl_TileDrivers.cdb_JP2Driver = GetGDALDriverManager()->GetDriverByName(cdb_JP2DriverNames[dcount].c_str());
		if (Gbl_TileDrivers.cdb_JP2Driver == NULL)
			++dcount;
		else if (Gbl_TileDrivers.cdb_JP2Driver->pfnOpen == NULL)
		{
			Gbl_TileDrivers.cdb_JP2Driver = NULL;
			++dcount;
		}
	}
	if (Gbl_TileDrivers.cdb_JP2Driver == NULL)
	{
		ErrorMsg = "No GDAL JP2 Driver Found";
		return false;
	}

	//Get the GeoTiff driver for the Elevation data
	Gbl_TileDrivers.cdb_GTIFFDriver = GetGDALDriverManager()->GetDriverByName("GTiff");
	if (Gbl_TileDrivers.cdb_GTIFFDriver == NULL)
	{
		ErrorMsg = "GDAL GeoTiff Driver Not Found";
		return false;
	}
	else if (Gbl_TileDrivers.cdb_GTIFFDriver->pfnOpen == NULL)
	{
		ErrorMsg = "GDAL GeoTiff Driver has no open function";
		return false;
	}

	//The Erdas Imagine dirver is currently being used for the
	//Elevation cache files
	Gbl_TileDrivers.cdb_HFADriver = GetGDALDriverManager()->GetDriverByName("HFA");
	if (Gbl_TileDrivers.cdb_HFADriver == NULL)
	{
		ErrorMsg = "GDAL ERDAS Imagine Driver Not Found";
		return false;
	}
	else if (Gbl_TileDrivers.cdb_HFADriver->pfnOpen == NULL)
	{
		ErrorMsg = "GDAL ERDAS Imagine Driver has no open function";
		return false;
	}

	Gbl_TileDrivers.cdb_ShapefileDriver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
	if (Gbl_TileDrivers.cdb_ShapefileDriver == NULL)
	{
		ErrorMsg = "GDAL ESRI Shapefile Driver Not Found";
		return false;
	}
	else if (Gbl_TileDrivers.cdb_ShapefileDriver->pfnOpen == NULL)
	{
		ErrorMsg = "GDAL ESRI Shapefile Driver has no open function";
		return false;
	}

	Gbl_TileDrivers.cdb_GeoPackageDriver = GetGDALDriverManager()->GetDriverByName("GPKG");
	if (Gbl_TileDrivers.cdb_GeoPackageDriver == NULL)
	{
		ErrorMsg = "GeoPackageDriver Driver Not Found";
		return false;
	}
	else if (Gbl_TileDrivers.cdb_GeoPackageDriver->pfnOpen == NULL)
	{
		ErrorMsg = "GDAL GeoPackageDriver Driver has no open function";
		return false;
	}

	Gbl_TileDrivers.cdb_drivers_initialized = true;
	return true;
}

Image_Contrib CDB_Tile::Get_Contribution(CDB_Tile_Extent &TileExtent)
{
	//Does the Image fall entirly within the Tile
	Image_Contrib retval = Image_Is_Inside_Tile(TileExtent);
	if (retval != None)
		return retval;

	//Check the tile contents against the Image
	if ((TileExtent.North < m_TileExtent.South) || (TileExtent.South > m_TileExtent.North) ||
		(TileExtent.East < m_TileExtent.West) || (TileExtent.West > m_TileExtent.East))
		retval = None;
	else if ((TileExtent.North <= m_TileExtent.North) && (TileExtent.South >= m_TileExtent.South) &&
		(TileExtent.East <= m_TileExtent.East) && (TileExtent.West >= m_TileExtent.West))
	{
		retval = Full;
	}
	else
		retval = Partial;

	return retval;
}

Image_Contrib CDB_Tile::Image_Is_Inside_Tile(CDB_Tile_Extent &TileExtent)
{
	int count = 0;
	coord2d point;
	point.Xpos = m_TileExtent.West;
	point.Ypos = m_TileExtent.North;
	if (Point_is_Inside_Tile(point, TileExtent))
		++count;

	point.Xpos = m_TileExtent.East;
	if (Point_is_Inside_Tile(point, TileExtent))
		++count;

	point.Ypos = m_TileExtent.South;
	if (Point_is_Inside_Tile(point, TileExtent))
		++count;

	point.Xpos = m_TileExtent.West;
	if (Point_is_Inside_Tile(point, TileExtent))
		++count;

	if (count > 0)
		return Partial;
	else
		return None;
}

bool CDB_Tile::Point_is_Inside_Tile(coord2d &Point, CDB_Tile_Extent &TileExtent)
{
	if ((Point.Xpos < TileExtent.East) && (Point.Xpos > TileExtent.West) && (Point.Ypos > TileExtent.South) && (Point.Ypos < TileExtent.North))
		return true;
	else
		return false;
}

bool CDB_Tile::Load_Tile(void)
{
	if (m_Tile_Status == Loaded)
		return true;

	if (!m_FileExists)
		return false;

	Allocate_Buffers();

	if (!Open_Tile())
		return false;

	if (!Read())
		return false;

	return true;
}

coord2d CDB_Tile::LL2Pix(coord2d LLPoint)
{
	coord2d PixCoord;
	if ((m_Tile_Status == Loaded) || (m_Tile_Status == Opened))
	{
		double xRel = LLPoint.Xpos - m_TileExtent.West;
		double yRel = m_TileExtent.North - LLPoint.Ypos;
		PixCoord.Xpos = xRel / m_GDAL.adfGeoTransform[GEOTRSFRM_WE_RES];
		PixCoord.Ypos = yRel / abs(m_GDAL.adfGeoTransform[GEOTRSFRM_NS_RES]);
	}
	else
	{
		PixCoord.Xpos = -1.0;
		PixCoord.Ypos = -1.0;
	}
	return PixCoord;
}

bool CDB_Tile::Get_Image_Pixel(coord2d ImPix, unsigned char &RedPix, unsigned char &GreenPix, unsigned char &BluePix)
{
	int tx = (int)ImPix.Xpos;
	int ty = (int)ImPix.Ypos;

	if ((tx < 0) || (tx > m_Pixels.pixX - 1) || (ty < 0) || (ty > m_Pixels.pixY - 1))
	{
		return false;
	}

	int bpos1 = (((int)ImPix.Ypos) * m_Pixels.pixX) + (int)ImPix.Xpos;
	int bpos2 = bpos1 + 1;
	int bpos3 = bpos1 + (int)m_Pixels.pixX;
	int bpos4 = bpos3 + 1;

	if (tx == m_Pixels.pixX - 1)
	{
		bpos2 = bpos1;
		bpos4 = bpos3;
	}

	if (ty == m_Pixels.pixY - 1)
	{
		bpos3 = bpos1;
		if (tx == m_Pixels.pixX - 1)
			bpos4 = bpos1;
		else
			bpos4 = bpos2;
	}

	float rat2 = (float)(ImPix.Xpos - double(tx));
	float rat1 = 1.0f - rat2;
	float rat4 = (float)(ImPix.Ypos - double(ty));
	float rat3 = 1.0f - rat4;

	float p1p = ((float)m_GDAL.reddata[bpos1] * rat1) + ((float)m_GDAL.reddata[bpos2] * rat2);
	float p2p = ((float)m_GDAL.reddata[bpos3] * rat1) + ((float)m_GDAL.reddata[bpos4] * rat2);
	float red = (p1p * rat3) + (p2p * rat4);

	p1p = ((float)m_GDAL.greendata[bpos1] * rat1) + ((float)m_GDAL.greendata[bpos2] * rat2);
	p2p = ((float)m_GDAL.greendata[bpos3] * rat1) + ((float)m_GDAL.greendata[bpos4] * rat2);
	float green = (p1p * rat3) + (p2p * rat4);

	p1p = ((float)m_GDAL.bluedata[bpos1] * rat1) + ((float)m_GDAL.bluedata[bpos2] * rat2);
	p2p = ((float)m_GDAL.bluedata[bpos3] * rat1) + ((float)m_GDAL.bluedata[bpos4] * rat2);
	float blue = (p1p * rat3) + (p2p * rat4);

	red = round(red) < 255.0f ? round(red) : 255.0f;
	green = round(green) < 255.0f ? round(green) : 255.0f;
	blue = round(blue) < 255.0f ? round(blue) : 255.0f;

	RedPix = (unsigned char)red;
	GreenPix = (unsigned char)green;
	BluePix = (unsigned char)blue;
	return true;

}

bool CDB_Tile::Get_Elevation_Pixel(coord2d ImPix, float &ElevationPix)
{
	int tx = (int)ImPix.Xpos;
	int ty = (int)ImPix.Ypos;

	if ((tx < 0) || (tx > m_Pixels.pixX - 1) || (ty < 0) || (ty > m_Pixels.pixY - 1))
	{
		return false;
	}

	int bpos1 = (((int)ImPix.Ypos) * m_Pixels.pixX) + (int)ImPix.Xpos;
	int bpos2 = bpos1 + 1;
	int bpos3 = bpos1 + (int)m_Pixels.pixX;
	int bpos4 = bpos3 + 1;

	if (tx == m_Pixels.pixX - 1)
	{
		bpos2 = bpos1;
		bpos4 = bpos3;
	}

	if (ty == m_Pixels.pixY - 1)
	{
		bpos3 = bpos1;
		if (tx == m_Pixels.pixX - 1)
			bpos4 = bpos1;
		else
			bpos4 = bpos2;
	}

	float rat2 = (float)(ImPix.Xpos - double(tx));
	float rat1 = 1.0f - rat2;
	float rat4 = (float)(ImPix.Ypos - double(ty));
	float rat3 = 1.0f - rat4;

	float e1p = (m_GDAL.elevationdata[bpos1] * rat1) + (m_GDAL.elevationdata[bpos2] * rat2);
	float e2p = (m_GDAL.elevationdata[bpos3] * rat1) + (m_GDAL.elevationdata[bpos4] * rat2);
	float elevation = (e1p * rat3) + (e2p * rat4);

	ElevationPix = elevation;
	return true;
}

bool CDB_Tile::Get_Subordinate_Elevation_Pixel(coord2d ImPix, float &ElevationPix)
{
	int tx = (int)ImPix.Xpos;
	int ty = (int)ImPix.Ypos;

	if ((tx < 0) || (tx > m_Pixels.pixX - 1) || (ty < 0) || (ty > m_Pixels.pixY - 1))
	{
		return false;
	}

	int bpos1 = (((int)ImPix.Ypos) * m_Pixels.pixX) + (int)ImPix.Xpos;
	int bpos2 = bpos1 + 1;
	int bpos3 = bpos1 + (int)m_Pixels.pixX;
	int bpos4 = bpos3 + 1;

	if (tx == m_Pixels.pixX - 1)
	{
		bpos2 = bpos1;
		bpos4 = bpos3;
	}

	if (ty == m_Pixels.pixY - 1)
	{
		bpos3 = bpos1;
		if (tx == m_Pixels.pixX - 1)
			bpos4 = bpos1;
		else
			bpos4 = bpos2;
	}

	float rat2 = (float)(ImPix.Xpos - double(tx));
	float rat1 = 1.0f - rat2;
	float rat4 = (float)(ImPix.Ypos - double(ty));
	float rat3 = 1.0f - rat4;

	float e1p = (m_GDAL.subord_elevationdata[bpos1] * rat1) + (m_GDAL.subord_elevationdata[bpos2] * rat2);
	float e2p = (m_GDAL.subord_elevationdata[bpos3] * rat1) + (m_GDAL.subord_elevationdata[bpos4] * rat2);
	float elevation = (e1p * rat3) + (e2p * rat4);

	ElevationPix = elevation;
	return true;
}

double CDB_Tile::West(void)
{
	return m_TileExtent.West;
}

double CDB_Tile::East(void)
{
	return m_TileExtent.East;
}

double CDB_Tile::North(void)
{
	return m_TileExtent.North;
}

double CDB_Tile::South(void)
{
	return m_TileExtent.South;
}

void CDB_Tile::Build_From_Tiles(CDB_TilePV *Tiles, bool from_scratch)
{

	for each (CDB_TileP tile in *Tiles)
	{
		if (tile->Subordinate_Exists())
		{
			m_Subordinate_Tile = true;
			break;
		}
	}

	Allocate_Buffers();

	if (!from_scratch && m_FileExists)
	{
		Open_Tile();
		//Load the current tile Information
		Read();
	}
	else
	{
		Fill_Tile();
	}

	bool have_some_contribution = false;
	double XRes = (m_TileExtent.East - m_TileExtent.West) / (double)m_Pixels.pixX;
	double YRes = (m_TileExtent.North - m_TileExtent.South) / (double)m_Pixels.pixY;
	for each (CDB_TileP tile in *Tiles)
	{
		Image_Contrib ImageContrib = tile->Get_Contribution(m_TileExtent);
		if ((ImageContrib == Full) || (ImageContrib == Partial))
		{
			if (tile->Load_Tile())
			{
				have_some_contribution = true;
				int sy = (int)((m_TileExtent.North - tile->North()) / YRes);
				if (sy < 0)
					sy = 0;
				int ey = (int)((m_TileExtent.North - tile->South()) / YRes);
				if (ey > m_Pixels.pixY - 1)
					ey = m_Pixels.pixY - 1;
				int sx = (int)((tile->West() - m_TileExtent.West) / XRes);
				if (sx < 0)
					sx = 0;
				int ex = (int)((tile->East() - m_TileExtent.West) / XRes);
				if (ex > m_Pixels.pixX - 1)
					ex = m_Pixels.pixX - 1;

				double srowlon = m_TileExtent.West + ((double)sx * XRes);
				double srowlat = m_TileExtent.North - ((double)sy *  YRes);
				int buffloc = 0;
				coord2d clatlon;
				clatlon.Ypos = srowlat;
				bool proces_subordinate = m_Subordinate_Tile && tile->Subordinate_Exists();
				for (int iy = sy; iy <= ey; ++iy)
				{
					buffloc = (iy * m_Pixels.pixX) + sx;
					clatlon.Xpos = srowlon;
					for (int ix = sx; ix <= ex; ++ix)
					{
						coord2d impix = tile->LL2Pix(clatlon);
						if ((m_TileType == Imagery) || (m_TileType == ImageryCache))
						{
							unsigned char redpix, greenpix, bluepix;
							if (tile->Get_Image_Pixel(impix, redpix, greenpix, bluepix))
							{
								m_GDAL.reddata[buffloc] = redpix;
								m_GDAL.greendata[buffloc] = greenpix;
								m_GDAL.bluedata[buffloc] = bluepix;
							}
						}
						else if ((m_TileType == Elevation) || (m_TileType == ElevationCache))
						{
							tile->Get_Elevation_Pixel(impix, m_GDAL.elevationdata[buffloc]);
							if(proces_subordinate)
								tile->Get_Subordinate_Elevation_Pixel(impix, m_GDAL.subord_elevationdata[buffloc]);
						}
						++buffloc;
						clatlon.Xpos += XRes;
					}
					clatlon.Ypos -= YRes;
				}
			}
			tile->Free_Resources();

		}
	}

	if (have_some_contribution)
	{
		m_Tile_Status = Loaded;
	}

}

bool CDB_Tile::Save(void)
{
	char **papszOptions = NULL;

	//Set the transformation Matrix
	m_GDAL.adfGeoTransform[0] = m_TileExtent.West;
	m_GDAL.adfGeoTransform[1] = (m_TileExtent.East - m_TileExtent.West) / (double)m_Pixels.pixX;
	m_GDAL.adfGeoTransform[2] = 0.0;
	m_GDAL.adfGeoTransform[3] = m_TileExtent.North;
	m_GDAL.adfGeoTransform[4] = 0.0;
	m_GDAL.adfGeoTransform[5] = ((m_TileExtent.North - m_TileExtent.South) / (double)m_Pixels.pixY) * -1.0;

	OGRSpatialReference *CDB_SRS = new OGRSpatialReference();
	CDB_SRS->SetWellKnownGeogCS("WGS84");
	if (m_GDAL.poDataset)
	{
		Close_Dataset();
	}


	if (m_TileType == ImageryCache)
	{
		if (m_GDAL.poDataset == NULL)
		{
			//Get the Imagine Driver
			m_GDAL.poDriver = Gbl_TileDrivers.cdb_GTIFFDriver;
			GDALDataType dataType = GDT_Byte;
			if (!m_GDAL.poDriver)
			{
				delete CDB_SRS;
				return false;
			}
			//Create the file
			m_GDAL.poDataset = m_GDAL.poDriver->Create(m_FileName.c_str(), m_Pixels.pixX, m_Pixels.pixY, m_Pixels.bands, dataType, papszOptions);

			if (!m_GDAL.poDataset)
			{
				delete CDB_SRS;
				return false;
			}

			m_GDAL.poDataset->SetGeoTransform(m_GDAL.adfGeoTransform);
			char *projection = NULL;
			CDB_SRS->exportToWkt(&projection);
			m_GDAL.poDataset->SetProjection(projection);
			CPLFree(projection);
		}
		//Write the elevation data to the file
		if (!Write())
		{
			delete CDB_SRS;
			return false;
		}
	}
	else if (m_TileType == ElevationCache)
	{
		if (m_GDAL.poDataset == NULL)
		{
			//Get the Imagine Driver
			m_GDAL.poDriver = Gbl_TileDrivers.cdb_HFADriver;
			GDALDataType dataType = GDT_Float32;
			if (!m_GDAL.poDriver)
			{
				delete CDB_SRS;
				return false;
			}
			//Create the file
			m_GDAL.poDataset = m_GDAL.poDriver->Create(m_FileName.c_str(), m_Pixels.pixX, m_Pixels.pixY, m_Pixels.bands, dataType, papszOptions);

			if (!m_GDAL.poDataset)
			{
				delete CDB_SRS;
				return false;
			}
			m_GDAL.poDataset->SetGeoTransform(m_GDAL.adfGeoTransform);
			char *projection = NULL;
			CDB_SRS->exportToWkt(&projection);
			m_GDAL.poDataset->SetProjection(projection);
			if (m_Subordinate_Tile)
			{
				m_GDAL.soDataset = m_GDAL.poDriver->Create(m_SubordinateName.c_str(), m_Pixels.pixX, m_Pixels.pixY, m_Pixels.bands, dataType, papszOptions);

				if (!m_GDAL.soDataset)
				{
					delete CDB_SRS;
					return false;
				}
				m_GDAL.soDataset->SetGeoTransform(m_GDAL.adfGeoTransform);
				m_GDAL.poDataset->SetProjection(projection);
			}
			CPLFree(projection);
		}
		//Write the elevation data to the file
		if (!Write())
		{
			delete CDB_SRS;
			return false;
		}
	}

	delete CDB_SRS;
	return true;
}

bool CDB_Tile::Write(void)
{
	CPLErr gdal_err;

	if (m_TileType == ImageryCache)
	{
		GDALRasterBand * RedBand = m_GDAL.poDataset->GetRasterBand(1);
		GDALRasterBand * GreenBand = m_GDAL.poDataset->GetRasterBand(2);
		GDALRasterBand * BlueBand = m_GDAL.poDataset->GetRasterBand(3);

		gdal_err = RedBand->RasterIO(GF_Write, 0, 0, m_Pixels.pixX, m_Pixels.pixY, m_GDAL.reddata, m_Pixels.pixX, m_Pixels.pixY, GDT_Byte, 0, 0);
		if (gdal_err == CE_Failure)
		{
			return false;
		}

		gdal_err = GreenBand->RasterIO(GF_Write, 0, 0, m_Pixels.pixX, m_Pixels.pixY, m_GDAL.greendata, m_Pixels.pixX, m_Pixels.pixY, GDT_Byte, 0, 0);
		if (gdal_err == CE_Failure)
		{
			return false;
		}

		gdal_err = BlueBand->RasterIO(GF_Write, 0, 0, m_Pixels.pixX, m_Pixels.pixY, m_GDAL.bluedata, m_Pixels.pixX, m_Pixels.pixY, GDT_Byte, 0, 0);
		if (gdal_err == CE_Failure)
		{
			return false;
		}
	}
	else if (m_TileType == ElevationCache)
	{
		GDALRasterBand * ElevationBand = m_GDAL.poDataset->GetRasterBand(1);
		gdal_err = ElevationBand->RasterIO(GF_Write, 0, 0, m_Pixels.pixX, m_Pixels.pixY, m_GDAL.elevationdata, m_Pixels.pixX, m_Pixels.pixY, GDT_Float32, 0, 0);
		if (gdal_err == CE_Failure)
		{
			return false;
		}
		if (m_Subordinate_Tile)
		{
			GDALRasterBand * SoElevationBand = m_GDAL.soDataset->GetRasterBand(1);
			gdal_err = SoElevationBand->RasterIO(GF_Write, 0, 0, m_Pixels.pixX, m_Pixels.pixY, m_GDAL.subord_elevationdata, m_Pixels.pixX, m_Pixels.pixY, GDT_Float32, 0, 0);
			if (gdal_err == CE_Failure)
			{
				return false;
			}
		}
	}
	return true;
}

osg::Image* CDB_Tile::Image_From_Tile(void)
{
	if (m_Tile_Status == Loaded)
	{
		//allocate the osg image
		osg::ref_ptr<osg::Image> image = new osg::Image;
		GLenum pixelFormat = GL_RGBA;
		image->allocateImage(m_Pixels.pixX, m_Pixels.pixY, 1, pixelFormat, GL_UNSIGNED_BYTE);
		memset(image->data(), 0, image->getImageSizeInBytes());
		int ibufpos = 0;
		int dst_row = 0;
		for (int iy = 0; iy < m_Pixels.pixY; ++iy)
		{
			int dst_col = 0;
			for (int ix = 0; ix < m_Pixels.pixX; ++ix)
			{
				//Populate the osg:image
				*(image->data(dst_col, dst_row) + 0) = m_GDAL.reddata[ibufpos];
				*(image->data(dst_col, dst_row) + 1) = m_GDAL.greendata[ibufpos];
				*(image->data(dst_col, dst_row) + 2) = m_GDAL.bluedata[ibufpos];
				*(image->data(dst_col, dst_row) + 3) = 255;
				++ibufpos;
				++dst_col;
			}
			++dst_row;
		}
		image->flipVertical();
		return image.release();
	}
	else
		return NULL;
}

osg::HeightField* CDB_Tile::HeightField_From_Tile(void)
{
	if (m_Tile_Status == Loaded)
	{
		osg::ref_ptr<osg::HeightField> field = new osg::HeightField;
		field->allocate(m_Pixels.pixX, m_Pixels.pixY);
		//For now clear the data
		for (unsigned int i = 0; i < field->getHeightList().size(); ++i)
			field->getHeightList()[i] = NO_DATA_VALUE;

		int ibpos = 0;
		for (unsigned int r = 0; r < (unsigned)m_Pixels.pixY; r++)
		{
			unsigned inv_r = m_Pixels.pixY - r - 1;
			for (unsigned int c = 0; c < (unsigned)m_Pixels.pixX; c++)
			{
				//Re-enable this if data is suspect
				//Should already be filtered in CDB creation however
#if 0
				float h = elevation[ibpos];
				// Mark the value as nodata using the universal NO_DATA_VALUE marker.
				if (!isValidValue(h, band))
				{
					h = NO_DATA_VALUE;
				}
#endif
				if(m_Subordinate_Exists || m_Subordinate_Tile)
					field->setHeight(c, inv_r, (m_GDAL.elevationdata[ibpos] - m_GDAL.subord_elevationdata[ibpos]));
				else
					field->setHeight(c, inv_r, m_GDAL.elevationdata[ibpos]);
				++ibpos;
			}
		}
		return field.release();
	}
	else
		return NULL;
}

std::string CDB_Tile::Xml_Name(std::string Name)
{
	std::string retString = "";
	size_t pos = Name.find_last_of(".");
	if (pos != string::npos)
	{
		retString = Name.substr(0, pos) + ".xml";
	}
	return retString;
}

std::string CDB_Tile::Set_FileType(std::string Name, std::string type)
{
	std::string retString = "";
	size_t pos = Name.find_last_of(".");
	if (pos != string::npos)
	{
		retString = Name.substr(0, pos) + type;
	}
	return retString;

}

std::string CDB_Tile::Model_FullFileName(std::string &FACC_value, std::string &FSC_value, std::string &BaseFileName)
{
	std::stringstream modbuf;
	modbuf << m_cdbRootDir
		<< "\\Tiles"
		<< "\\" << m_lat_str
		<< "\\" << m_lon_str
		<< "\\300_GSModelGeometry"
		<< "\\" << m_lod_str
		<< "\\" << m_uref_str
		<< "\\" << m_lat_str << m_lon_str << "_D300_S001_T001_" << m_lod_str
		<< "_" << m_uref_str << "_" << m_rref_str << "_"
		<< FACC_value << "_" << FSC_value << "_"
		<< BaseFileName << ".flt";
	return modbuf.str();
}

std::string CDB_Tile::GeoTypical_FullFileName(std::string &BaseFileName)
{
	std::string Facc1 = BaseFileName.substr(0, 1);
	std::string Facc2 = BaseFileName.substr(1, 1);
	std::string Fcode = BaseFileName.substr(2, 3);

	//First Level subdirectory
	if (Facc1 == "A")
		Facc1 = "A_Culture";
	else if (Facc1 == "E")
		Facc1 = "E_Vegetation";
	else if (Facc1 == "B")
		Facc1 = "B_Hydrography";
	else if (Facc1 == "C")
		Facc1 = "C_Hypsography";
	else if (Facc1 == "D")
		Facc1 = "D_Physiography";
	else if (Facc1 == "F")
		Facc1 = "F_Demarcation";
	else if (Facc1 == "G")
		Facc1 = "G_Aeronautical_Information";
	else if (Facc1 == "I")
		Facc1 = "I_Cadastral";
	else if (Facc1 == "S")
		Facc1 = "S_Special_Use";
	else
		Facc1 = "Z_General";

	//Second Level Directory
	if (Facc2 == "L")
		Facc2 = "L_Misc_Feature";
	else if (Facc2 == "T")
		Facc2 = "T_Comm";
	else if (Facc2 == "C")
		Facc2 = "C_Woodland";
	else if (Facc2 == "K")
		Facc2 = "K_Recreational";

	if (Facc1 == "A_Culture")
	{
		if (Fcode == "050")
			Fcode = "050_Display_Sign";
		else if (Fcode == "110")
			Fcode = "110_Light_Standard";
		else if (Fcode == "030")
			Fcode = "030_Power_Line";
	}
	else if (Facc1 == "E_Vegetation")
	{
		if (Fcode == "030")
			Fcode = "030_Trees";
	}

	std::stringstream modbuf;
	modbuf << m_cdbRootDir
		<< "\\GTModel\\500_GTModelGeometry"
		<< "\\" << Facc1
		<< "\\" << Facc2
		<< "\\" << Fcode
		<< "\\D500_S001_T001_" << BaseFileName;

	return modbuf.str();
}

std::string CDB_Tile::Model_FileName(std::string &FACC_value, std::string &FSC_value, std::string &BaseFileName)
{
	std::stringstream modbuf;
	modbuf << m_lat_str << m_lon_str << "_D300_S001_T001_" << m_lod_str
		<< "_" << m_uref_str << "_" << m_rref_str << "_"
		<< FACC_value << "_" << FSC_value << "_"
		<< BaseFileName << ".flt";
	return modbuf.str();
}

std::string CDB_Tile::Model_TextureDir(void)
{
	std::stringstream modbuf;
	modbuf << m_cdbRootDir
		<< "\\Tiles"
		<< "\\" << m_lat_str
		<< "\\" << m_lon_str
		<< "\\301_GSModelTexture"
		<< "\\" << m_lod_str
		<< "\\" << m_uref_str;
	return modbuf.str();
}

std::string CDB_Tile::Model_ZipDir(void)
{
	std::string retstr = "";
	if (m_TileType != GeoSpecificModel)
		return retstr;
	std::stringstream modbuf;
	modbuf << m_cdbRootDir
		<< "\\Tiles"
		<< "\\" << m_lat_str
		<< "\\" << m_lon_str
		<< "\\300_GSModelGeometry"
		<< "\\" << m_lod_str
		<< "\\" << m_uref_str;
	return modbuf.str();
}

bool CDB_Tile::validate_tile_name(std::string &filename)
{
#ifdef _WIN32
	DWORD ftyp = ::GetFileAttributes(filename.c_str());
	if (ftyp == INVALID_FILE_ATTRIBUTES)
	{
		DWORD error = ::GetLastError();
		if (error == ERROR_FILE_NOT_FOUND || error == ERROR_PATH_NOT_FOUND)
		{
			return false;
		}
		else
			return false;
	}
	return true;
#else
	int ftyp = ::access(filename.c_str(), F_OK);
	if (ftyp == 0)
	{
		return  true;
	}
	else
	{
		return false;
	}
#endif
}

std::string CDB_Tile::Model_HeaderName(void)
{
	std::string retstr = "";
	if (m_TileType != GeoSpecificModel && m_TileType != GeoTypicalModel)
		return retstr;

	std::stringstream modbuf;
	modbuf << m_lat_str << m_lon_str << "_D300_S001_T001_" << m_lod_str
		<< "_" << m_uref_str << "_" << m_rref_str << "_";
	return modbuf.str();

}

std::string CDB_Tile::Model_KeyNameFromArchiveName(const std::string &ArchiveFileName, std::string &Header)
{
	std::string KeyName = "";
	std::string SearchStr;
	//Mask off the selectors
	int spos1 = Header.find("_S");
	if (spos1 != std::string::npos)
		SearchStr = Header.substr(0, spos1 + 2);
	else
		SearchStr = Header;

	int pos1 = ArchiveFileName.find(SearchStr);
	if (pos1 != std::string::npos)
	{
		int pos2 = pos1 + Header.length();
		if (pos2 < ArchiveFileName.length())
		{
			KeyName = ArchiveFileName.substr(pos2);
		}
	}
	return KeyName;

}

osgDB::Archive::FileNameList * CDB_Tile::Model_Archive_List(void)
{
	if (m_TileType != GeoSpecificModel)
		return NULL;
	return &m_ModelSet.archiveFileList;
}

OGR_File::OGR_File() : m_FileName(""), m_Driver(""), m_oSRS(NULL), m_OutputIsShape(false), m_PODataset(NULL), m_FID(0), m_FileExists(false)
{
}

OGR_File::OGR_File(std::string FileName, std::string Driver) :m_FileName(FileName), m_Driver(Driver), m_oSRS(NULL), m_OutputIsShape(false), m_PODataset(NULL), m_FID(0)
{
	m_FileExists = CDB_Tile::validate_tile_name(m_FileName);
}

OGR_File::~OGR_File()
{

}

bool OGR_File::SetName_and_Driver(std::string Name, std::string Driver)
{
	m_FileName = Name;
	m_Driver = Driver;
	m_FileExists = CDB_Tile::validate_tile_name(m_FileName);
	return m_FileExists;
}
OGR_File * OGR_File::GetInstance(void)
{
	return &Ogr_File_Instance;
}

bool OGR_File::Exists(void)
{
	return m_FileExists;
}

GDALDataset * OGR_File::Open_Output_File(std::string FileName, bool FileExists)
{
#if 0
	while (!m_OpenLock->Lock())
	{
		Sleep(1000);
	}
#endif

	if (m_Driver == "ESRI Shapefile")
		m_OutputIsShape = true;

	GDALDataset *poDS = NULL;
	GDALDriver * poDriver = GetGDALDriverManager()->GetDriverByName(m_Driver.c_str());

	if (poDriver != NULL)
	{
		if (!FileExists)
		{
			poDS = poDriver->Create(FileName.c_str(), 0, 0, 0, GDT_Unknown, NULL);
			if (poDS == NULL)
			{
				std::string Message = "Unable to create output file " + FileName;
				printf("%s\n", Message.c_str());
#if 0
				m_OpenLock->Unlock();
#endif
				return NULL;
			}
		}
		else
		{
			GDALOpenInfo poOpenInfo(FileName.c_str(), GA_Update | GDAL_OF_VECTOR);
			poDS = poDriver->pfnOpen(&poOpenInfo);
			if (poDS == NULL)
			{
				std::string Message = "Unable to open output file " + FileName;
				printf("%s\n", Message.c_str());
#if 0
				m_OpenLock->Unlock();
#endif
				return NULL;
			}
		}
		if (m_oSRS == NULL)
		{
			Set_oSRS();
		}
	}
	else
	{
		std::string Message = "Unable to obtain driver for " + m_Driver;
		printf("%s\n", Message.c_str());
#if 0
		m_OpenLock->Unlock();
#endif
		return NULL;
	}

#if 0
	m_OpenLock->Unlock();
#endif

	return poDS;
}

void OGR_File::Set_oSRS(void)
{
	if (m_oSRS == NULL)
	{
		m_oSRS = new OGRSpatialReference();
		m_oSRS->SetWellKnownGeogCS("WGS84");
	}
}

GDALDataset * OGR_File::Open_Output(void)
{
	m_PODataset = Open_Output_File(m_FileName, m_FileExists);
	return m_PODataset;
}

OGRLayer * OGR_File::Get_Or_Create_Layer(std::string LayerName, osgEarth::Features::Feature * f)
{
	return Get_Or_Create_Layer(m_PODataset, LayerName, f);
}

OGRLayer * OGR_File::Get_Or_Create_Layer(GDALDataset *poDS, std::string LayerName, osgEarth::Features::Feature * f)
{
	OGRLayer *poLayer = NULL;
	if (poDS->TestCapability(ODsCCreateLayer))
	{
		poLayer = poDS->GetLayerByName(LayerName.c_str());
		if (!poLayer)
		{
			OGRSpatialReference * LayerSRS = new OGRSpatialReference(*m_oSRS);
			osgEarth::Symbology::Geometry * geo = f->getGeometry();
			osgEarth::Symbology::Geometry::Type gtype = geo->getType();
			OGRwkbGeometryType OgrGtype = wkbPoint25D;
			if (gtype == osgEarth::Symbology::Geometry::Type::TYPE_POINTSET)
				OgrGtype = wkbPoint25D;
			else if (gtype == osgEarth::Symbology::Geometry::Type::TYPE_POLYGON)
				OgrGtype = wkbPolygon25D;
			else if (gtype == osgEarth::Symbology::Geometry::Type::TYPE_LINESTRING)
				OgrGtype = wkbLineString25D;
			poLayer = poDS->CreateLayer(LayerName.c_str(), LayerSRS, OgrGtype, NULL);
			if (poLayer)
			{
				if (!Set_Layer_Fields(f, poLayer))
				{
					std::string Message = "Failure to set Layer Feilds in " + m_FileName;
					printf("%s\n", Message.c_str());
					return NULL;
				}
			}
		}
		else
		{
			if (!Check_Layer_Fields(f, poLayer))
			{
				std::string Message = "Failure to set Layer Feilds in " + m_FileName;
				printf("%s\n", Message.c_str());
				return NULL;
			}

		}
	}
	else
	{
		std::string Message = "Unable to create layers in file " + m_FileName;
		printf("%s\n", Message.c_str());
		return NULL;
	}
	return poLayer;
}

bool OGR_File::Set_Layer_Fields(osgEarth::Features::Feature * f, OGRLayer * poLayer)
{
	osgEarth::Features::AttributeTable t = f->getAttrs();
	for (osgEarth::Features::AttributeTable::iterator ti = t.begin(); ti != t.end(); ++ti)
	{
		std::pair<std::string, osgEarth::Features::AttributeValue> temp;
		temp = *ti;
		std::string name = temp.first;
		osgEarth::Features::AttributeValue value = temp.second;
		osgEarth::Features::AttributeType attrType = value.first;
		OGRFieldType ogrType;
		if (attrType == osgEarth::Features::AttributeType::ATTRTYPE_BOOL)
		{
			ogrType = OFTInteger;
		}
		else if (attrType == osgEarth::Features::AttributeType::ATTRTYPE_DOUBLE)
		{
			ogrType = OFTReal;
		}
		else if (attrType == osgEarth::Features::AttributeType::ATTRTYPE_INT)
		{
			ogrType = OFTInteger;
		}
		else if (attrType == osgEarth::Features::AttributeType::ATTRTYPE_STRING)
		{
			ogrType = OFTString;
		}
		else if (attrType == osgEarth::Features::AttributeType::ATTRTYPE_UNSPECIFIED)
		{
			ogrType = OFTBinary;
		}

		OGRFieldDefn po_FieldDefn(name.c_str(), ogrType);
		if (poLayer->CreateField(&po_FieldDefn) != OGRERR_NONE)
		{
			std::stringstream	buf;
			buf << "Unable to create Feild " << name << " in " << poLayer->GetName();
			std::string Message = buf.str();
			printf("%s\n", Message.c_str());
			return false;
		}

	}
	return true;
}

bool OGR_File::Check_Layer_Fields(osgEarth::Features::Feature * f, OGRLayer * poLayer)
{
	OGRFeatureDefn *LayerDef = poLayer->GetLayerDefn();
	int Defncnt = LayerDef->GetFieldCount();
	bool retval = true;
	osgEarth::Features::AttributeTable t = f->getAttrs();
	for (osgEarth::Features::AttributeTable::iterator ti = t.begin(); ti != t.end(); ++ti)
	{
		std::pair<std::string, osgEarth::Features::AttributeValue> temp;
		temp = *ti;
		std::string name = temp.first;
		osgEarth::Features::AttributeValue value = temp.second;
		osgEarth::Features::AttributeType attrType = value.first;
		OGRFieldType ogrType;
		if (attrType == osgEarth::Features::AttributeType::ATTRTYPE_BOOL)
		{
			ogrType = OFTInteger;
		}
		else if (attrType == osgEarth::Features::AttributeType::ATTRTYPE_DOUBLE)
		{
			ogrType = OFTReal;
		}
		else if (attrType == osgEarth::Features::AttributeType::ATTRTYPE_INT)
		{
			ogrType = OFTInteger;
		}
		else if (attrType == osgEarth::Features::AttributeType::ATTRTYPE_STRING)
		{
			ogrType = OFTString;
		}
		else if (attrType == osgEarth::Features::AttributeType::ATTRTYPE_UNSPECIFIED)
		{
			ogrType = OFTBinary;
		}
		
		bool have_attr = false;
		bool add_attr = false;
		bool bad_attr = false;
		for (int i = 0; i < Defncnt; ++i)
		{
			OGRFieldDefn *po_FieldDefn = LayerDef->GetFieldDefn(i);
			std::string nameref = po_FieldDefn->GetNameRef();
			if (nameref == name)
			{
				if (ogrType == po_FieldDefn->GetType())
				{
					have_attr = true;
				}
				else
					bad_attr = true;
				break;
			}
		}
		if (!have_attr && !bad_attr)
		{
			OGRFieldDefn npo_FieldDefn(name.c_str(), ogrType);
			if (poLayer->CreateField(&npo_FieldDefn) != OGRERR_NONE)
			{
				std::stringstream	buf;
				buf << "Unable to create Feild " << name << " in " << poLayer->GetName();
				std::string Message = buf.str();
				printf("%s\n", Message.c_str());
				return false;
			}
		}
		if (bad_attr)
			retval = false;
	}
	return retval;
}

bool OGR_File::Add_Feature_to_Layer(OGRLayer *oLayer, osgEarth::Features::Feature * f)
{
	bool retval = true;
	OGRFeatureDefn *FeaDefn = oLayer->GetLayerDefn();
	OGRFeature * OFeature = OGRFeature::CreateFeature(FeaDefn);
	OFeature->SetFID(m_FID);
	++m_FID;
	for (int i = 0; i < FeaDefn->GetFieldCount(); ++i)
	{
		OGRFieldDefn * Fdefn = FeaDefn->GetFieldDefn(i);
		OGRFieldType ogrType = Fdefn->GetType();
		std::string FieldName = Fdefn->GetNameRef();
		if (ogrType == OFTInteger)
		{
			int intval = f->getInt(FieldName);
			OFeature->SetField(i, intval);
		}
		else if (ogrType == OFTReal)
		{
			double doubleval = f->getDouble(FieldName);
			OFeature->SetField(i, doubleval);
		}
		else if (ogrType == OFTString)
		{
			std::string stringval = f->getString(FieldName);
			OFeature->SetField(i, stringval.c_str());
		}

	}

	osgEarth::Symbology::Geometry * geo = f->getGeometry();
	osgEarth::Symbology::Geometry::Type gtype = geo->getType();
	OGRwkbGeometryType OgrGtype = wkbPoint25D;
	if (gtype == osgEarth::Symbology::Geometry::Type::TYPE_POINTSET)
		OgrGtype = wkbPoint25D;
	else if (gtype == osgEarth::Symbology::Geometry::Type::TYPE_POLYGON)
		OgrGtype = wkbPolygon25D;
	else if (gtype == osgEarth::Symbology::Geometry::Type::TYPE_LINESTRING)
		OgrGtype = wkbLineString25D;
	if (OgrGtype == wkbPoint25D)
	{
		OGRPoint poPoint;
		osg::Vec3d vec = geo->at(0);
		poPoint.setX(vec.x());
		poPoint.setY(vec.y());
		poPoint.setZ(vec.z());
		OFeature->SetGeometry(&poPoint);
	}
	else if (OgrGtype == wkbPolygon25D)
	{
		OGRPolygon poPolygon;
		OGRLinearRing poRing;
		for (int ii = 0; ii < geo->size(); ++ii)
		{
			OGRPoint poPoint;
			osg::Vec3d vec = geo->at(ii);
			poPoint.setX(vec.x());
			poPoint.setY(vec.y());
			poPoint.setZ(vec.z());
			poRing.addPoint(&poPoint);
		}
		poPolygon.addRing(&poRing);
		OFeature->SetGeometry(&poPolygon);
	}
	else if (OgrGtype == wkbLineString25D)
	{
		OGRLineString oLineString;
		for (int ii = 0; ii < geo->size(); ++ii)
		{
			OGRPoint poPoint;
			osg::Vec3d vec = geo->at(ii);
			poPoint.setX(vec.x());
			poPoint.setY(vec.y());
			poPoint.setZ(vec.z());
			oLineString.addPoint(&poPoint);
		}
		OFeature->SetGeometry(&oLineString);
	}
	OGRErr oErr = oLayer->CreateFeature(OFeature);
	if (oErr != OGRERR_NONE)
		retval = false;
	OGRFeature::DestroyFeature(OFeature);
	return retval;
}

bool OGR_File::Close_File(void)
{
	bool retval = false;
	if (m_PODataset)
	{
		GDALClose(m_PODataset);
		m_PODataset = NULL;
		m_FID = 0;
		m_FileName = "";
		m_FileExists = false;
		if (m_oSRS)
		{
			delete m_oSRS;
			m_oSRS = NULL;
		}
		retval = true;
	}
	return retval;
}
