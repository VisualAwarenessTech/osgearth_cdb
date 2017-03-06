/* -*-c++-*- */
//The CDBV driver is a plugin for the osgEarth Library
//Coppyright 2016-2017 Visual Awarness Technologies and Consulting Inc.
//
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2015 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <osgEarth/Registry>
#include <osgEarth/FileUtils>
#include <osgEarth/StringUtils>
#include <osgEarthFeatures/FeatureSource>
#include <osgEarthFeatures/Filter>
#include <osgEarthFeatures/BufferFilter>
#include <osgEarthFeatures/ScaleFilter>
#include <osgEarthFeatures/GeometryUtils>
#include "CDBVFeatureOptions"
#include "FeatureCursorCDBV"
#include <osgEarthFeatures/OgrUtils>
#include <osg/Notify>
#include <osgDB/FileNameUtils>
#include <osgDB/FileUtils>
#include <list>
#include <ogr_api.h>
#include <cpl_error.h>
#ifdef _WIN32
#include <windows.h>
#endif

#include <CDB_TileLib/CDB_Tile>
#include <vector>
#define LC "[CDBV FeatureSource] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Drivers;

#define OGR_SCOPED_LOCK GDAL_SCOPED_LOCK

namespace
{
    // helper function.
    OGRLayerH openLayer(OGRDataSourceH ds, const std::string& layer)
    {
        OGRLayerH h = OGR_DS_GetLayerByName(ds, layer.c_str());
        if ( !h )
        {
            unsigned index = osgEarth::as<unsigned>(layer, 0);
            h = OGR_DS_GetLayer(ds, index);
        }
        return h;
    }
}


/**
 * A FeatureSource that reads features from an OGR driver.
 *
 * This FeatureSource does NOT support styling.
 */
class CDBVFeatureSource : public FeatureSource
{
public:
    CDBVFeatureSource( const CDBVFeatureOptions& options ) : FeatureSource( options ),
      _dsHandle( 0L ),
      _layerHandle( 0L ),
      _ogrDriverHandle( 0L ),
      _options( options ),
      _featureCount(-1),
	  _cacheDir(""),
	  _dataSet("_S001_T001_"),
	  _rootString(""),
      _needsSync(false),
      _writable(false),
	  _Be_Verbose(false)
    {
        //nop
    }

    /** Destruct the object, cleaning up and OGR handles. */
    virtual ~CDBVFeatureSource()
    {       
        OGR_SCOPED_LOCK;

        if ( _layerHandle )
        {
            if (_needsSync)
            {
                OGR_L_SyncToDisk( _layerHandle ); // for writing only
                const char* name = OGR_FD_GetName( OGR_L_GetLayerDefn( _layerHandle ) );
                std::stringstream buf;
                buf << "REPACK " << name; 
                std::string bufStr;
                bufStr = buf.str();
                OE_DEBUG << LC << "SQL: " << bufStr << std::endl;
                OGR_DS_ExecuteSQL( _dsHandle, bufStr.c_str(), 0L, 0L );
            }
            _layerHandle = 0L;
        }

        if ( _dsHandle )
        {
            OGRReleaseDataSource( _dsHandle );
            _dsHandle = 0L;
        }
    }

    //override
    Status initialize( const osgDB::Options* dbOptions )
    {

		_dbOptions = dbOptions ? osg::clone(dbOptions) : 0L;

//		osgEarth::CachePolicy::NO_CACHE.apply(_dbOptions.get());
		//ToDo when working reenable  the cache disable for development 
		_LayerName = _options.layer().get();
#ifdef _DEBUG
		OE_INFO << LC << "Initializing Layer " << _LayerName << std::endl;
#endif

		FeatureProfile* Feature_Profile = NULL;
		const Profile * CDBFeatureProfile = NULL;

		if (_options.Edit_Support().isSet())
			_CDB_Edit_Support = _options.Edit_Support().value();

		if (_options.Verbose().isSet())
			_Be_Verbose = _options.Verbose().value();

		if (_Be_Verbose)
		{
			printf("Initializing CDBV Layer\n");
		}

		if (_options.Limits().isSet())
		{
			std::string cdbLimits = _options.Limits().value();
			double	min_lon,
				max_lon,
				min_lat,
				max_lat;

			int count = sscanf(cdbLimits.c_str(), "%lf,%lf,%lf,%lf", &min_lon, &min_lat, &max_lon, &max_lat);
			if (count == 4)
			{
				//CDB tiles always filter to geocell boundaries
				min_lon = round(min_lon);
				min_lat = round(min_lat);
				max_lat = round(max_lat);
				max_lon = round(max_lon);
				if ((max_lon > min_lon) && (max_lat > min_lat))
				{
					unsigned tiles_x = (unsigned)(max_lon - min_lon);
					unsigned tiles_y = (unsigned)(max_lat - min_lat);
					osg::ref_ptr<const SpatialReference> src_srs;
					src_srs = SpatialReference::create("EPSG:4326");
					CDBFeatureProfile = osgEarth::Profile::create(src_srs, min_lon, min_lat, max_lon, max_lat, tiles_x, tiles_y);

					//			   Below works but same as no limits
					//			   setProfile(osgEarth::Profile::create(src_srs, -180.0, -90.0, 180.0, 90.0, min_lon, min_lat, max_lon, max_lat, 90U, 45U));
				}
			}
			if (!CDBFeatureProfile)
				OE_WARN << "Invalid Limits received by CDB Driver: Not using Limits" << std::endl;

		}
		int minLod, maxLod;
		if (_options.minLod().isSet())
			minLod = _options.minLod().value();
		else
			minLod = 2;

		if (_options.maxLod().isSet())
		{
			maxLod = _options.maxLod().value();
			if (maxLod < minLod)
				minLod = maxLod;
		}
		else
			maxLod = minLod;

		// Always a WGS84 unprojected lat/lon profile.
		if (!CDBFeatureProfile)
			CDBFeatureProfile = osgEarth::Profile::create("EPSG:4326", "", 90U, 45U);

		Feature_Profile = new FeatureProfile(CDBFeatureProfile->getExtent());
		Feature_Profile->setTiled(true);
		// Should work for now 
		Feature_Profile->setFirstLevel(minLod);
		Feature_Profile->setMaxLevel(maxLod);
		Feature_Profile->setProfile(CDBFeatureProfile);

		// Make sure the root directory is set
		if (!_options.rootDir().isSet())
		{
			OE_WARN << "CDB root directory not set!" << std::endl;
		}
		else
		{
			_rootString = _options.rootDir().value();
		}
 
        if (Feature_Profile)
        {
            if ( _options.geoInterp().isSet() )
            {
				Feature_Profile->geoInterp() = _options.geoInterp().get();
            }
        }

		bool errorset = false;
		std::string Errormsg = "";
		if (!CDB_Tile::Initialize_Tile_Drivers(Errormsg))
		{
			errorset = true;
		}

		if (Feature_Profile)
		{
			setFeatureProfile(Feature_Profile);
		}
		else
		{
			return Status::Error(Status::ResourceUnavailable, "CDB_FeatureSourceV Failed to establish a valid feature profile");
		}
		return Status::OK();

    }


    //override
    FeatureCursor* createFeatureCursor( const Symbology::Query& query )
    {
        OGRDataSourceH dsHandle = 0L;
        OGRLayerH layerHandle = 0L;

		const osgEarth::TileKey key = query.tileKey().get();
		const GeoExtent key_extent = key.getExtent();
		CDB_Tile_Type tiletype = GeoPackageMap;
		CDB_Tile_Extent tileExtent(key_extent.north(), key_extent.south(), key_extent.east(), key_extent.west());

		CDB_Tile *mainTile = new CDB_Tile(_rootString, _cacheDir, tiletype, _dataSet, &tileExtent);
		std::string base = mainTile->FileName();
		bool have_file = mainTile->Tile_Exists();
		if (_Be_Verbose)
		{
			if (have_file)
				printf("Loading vector tile %s\n", base.c_str());
			else
				printf("No tile for %s\n", base.c_str());
		}
        // open the handles safely:
        if(have_file)
		{
            OGR_SCOPED_LOCK;

            // Each cursor requires its own DS handle so that multi-threaded access will work.
            // The cursor impl will dispose of the new DS handle.
            dsHandle = OGROpenShared( base.c_str(), 0, &_ogrDriverHandle );
            if ( dsHandle )
            {
                layerHandle = openLayer(dsHandle, _LayerName);
#ifdef _DEBUG
				OE_INFO << LC << "Loading Tile " << base << std::endl;
#endif
			}
        }

        if ( dsHandle && layerHandle )
        {
            // cursor is responsible for the OGR handles.
			FeatureCursorCDBV * fc = new FeatureCursorCDBV(
				dsHandle,
				layerHandle,
				this,
				getFeatureProfile(),
				query,
				getFilters());

			return fc;
        }
        else
        {
            if ( dsHandle )
            {
                OGR_SCOPED_LOCK;
                OGRReleaseDataSource( dsHandle );
            }

            return 0L;
        }
		delete mainTile;
    }

    virtual bool deleteFeature(FeatureID fid)
    {
        if (_writable && _layerHandle)
        {
            OGR_SCOPED_LOCK;
            if (OGR_L_DeleteFeature( _layerHandle, fid ) == OGRERR_NONE)
            {
                _needsSync = true;
                return true;
            }            
        }
        return false;
    }

    virtual int getFeatureCount() const
    {
        return _featureCount;
    }

    bool supportsGetFeature() const
    {
        return false;
    }

    virtual Feature* getFeature( FeatureID fid )
    {
        Feature* result = NULL;

        if ( !isBlacklisted(fid) )
        {
            OGR_SCOPED_LOCK;
            OGRFeatureH handle = OGR_L_GetFeature( _layerHandle, fid);
            if (handle)
            {
                result = OgrUtils::createFeature( handle, getFeatureProfile() );
                OGR_F_Destroy( handle );
            }
        }
        return result;
    }

    virtual bool isWritable() const
    {
        return _writable;
    }

    const FeatureSchema& getSchema() const
    {
        return _schema;
    } 

    virtual bool insertFeature(Feature* feature)
    {
        OGR_SCOPED_LOCK;
        OGRFeatureH feature_handle = OGR_F_Create( OGR_L_GetLayerDefn( _layerHandle ) );
        if ( feature_handle )
        {
            const AttributeTable& attrs = feature->getAttrs();

            // assign the attributes:
            int num_fields = OGR_F_GetFieldCount( feature_handle );
            for( int i=0; i<num_fields; i++ )
            {
                OGRFieldDefnH field_handle_ref = OGR_F_GetFieldDefnRef( feature_handle, i );
                std::string name = OGR_Fld_GetNameRef( field_handle_ref );
                int field_index = OGR_F_GetFieldIndex( feature_handle, name.c_str() );

                AttributeTable::const_iterator a = attrs.find( toLower(name) );
                if ( a != attrs.end() )
                {
                    switch( OGR_Fld_GetType(field_handle_ref) )
                    {
                    case OFTInteger:
                        OGR_F_SetFieldInteger( feature_handle, field_index, a->second.getInt(0) );
                        break;
                    case OFTReal:
                        OGR_F_SetFieldDouble( feature_handle, field_index, a->second.getDouble(0.0) );
                        break;
                    case OFTString:
                        OGR_F_SetFieldString( feature_handle, field_index, a->second.getString().c_str() );
                        break;
                    default:break;
                    }
                }
            }

            // assign the geometry:
            OGRFeatureDefnH def = ::OGR_L_GetLayerDefn( _layerHandle );

            OGRwkbGeometryType reported_type = OGR_FD_GetGeomType( def );

            OGRGeometryH ogr_geometry = OgrUtils::createOgrGeometry( feature->getGeometry(), reported_type );
            if ( OGR_F_SetGeometryDirectly( feature_handle, ogr_geometry ) != OGRERR_NONE )
            {
                OE_WARN << LC << "OGR_F_SetGeometryDirectly failed!" << std::endl;
            }

            if ( OGR_L_CreateFeature( _layerHandle, feature_handle ) != OGRERR_NONE )
            {
                //TODO: handle error better
                OE_WARN << LC << "OGR_L_CreateFeature failed!" << std::endl;
                OGR_F_Destroy( feature_handle );
                return false;
            }

            // clean up the feature
            OGR_F_Destroy( feature_handle );
        }
        else
        {
            //TODO: handle error better
            OE_WARN << LC << "OGR_F_Create failed." << std::endl;
            return false;
        }

        dirty();

        return true;
    }

    virtual osgEarth::Symbology::Geometry::Type getGeometryType() const
    {
        return _geometryType;
    }

protected:

    // parses an explicit WKT geometry string into a Geometry.
    Symbology::Geometry* parseGeometry( const Config& geomConf )
    {
        return GeometryUtils::geometryFromWKT( geomConf.value() );
    }

    // read the WKT geometry from a URL, then parse into a Geometry.
    Symbology::Geometry* parseGeometryUrl( const std::string& geomUrl, const osgDB::Options* dbOptions )
    {
        ReadResult r = URI(geomUrl).readString( dbOptions );
        if ( r.succeeded() )
        {
            Config conf( "geometry", r.getString() );
            return parseGeometry( conf );
        }
        return 0L;
    }

    void initSchema()
    {
        OGRFeatureDefnH layerDef =  OGR_L_GetLayerDefn( _layerHandle );
        for (int i = 0; i < OGR_FD_GetFieldCount( layerDef ); i++)
        {
            OGRFieldDefnH fieldDef = OGR_FD_GetFieldDefn( layerDef, i );
            std::string name;
            name = std::string( OGR_Fld_GetNameRef( fieldDef ) );
            OGRFieldType ogrType = OGR_Fld_GetType( fieldDef );
            _schema[ name ] = OgrUtils::getAttributeType( ogrType );
        }
    }




private:
    OGRDataSourceH _dsHandle;
    OGRLayerH _layerHandle;
    OGRSFDriverH _ogrDriverHandle;
#if 0
    osg::ref_ptr<Symbology::Geometry> _geometry; // explicit geometry.
#endif

    const CDBVFeatureOptions _options;
    int _featureCount;
    bool _needsSync;
    bool _writable;
    FeatureSchema _schema;
    Geometry::Type _geometryType;

	bool							_CDB_Edit_Support;
	osg::ref_ptr<CacheBin>          _cacheBin;
	osg::ref_ptr<osgDB::Options>    _dbOptions;
	int								_CDBLodNum;
	std::string						_rootString;
	std::string						_cacheDir;
	std::string						_dataSet;
	std::string						_lat_string;
	std::string						_lon_string;
	std::string						_uref_string;
	std::string						_rref_string;
	std::string						_lod_string;
	std::string						_LayerName;
	int								_cur_Feature_Cnt;
	bool							_Be_Verbose;

};


class CDBVFeatureSourceFactory : public FeatureSourceDriver
{
public:
    CDBVFeatureSourceFactory()
    {
        supportsExtension( "osgearth_feature_cdbv", "CDB Vector feature driver for osgEarth" );
    }

    virtual const char* className()
    {
        return "CDB Vector Feature Reader";
    }

    virtual ReadResult readObject(const std::string& file_name, const Options* options) const
    {
        if ( !acceptsExtension(osgDB::getLowerCaseFileExtension( file_name )))
            return ReadResult::FILE_NOT_HANDLED;

        return ReadResult( new CDBVFeatureSource( getFeatureSourceOptions(options) ) );
    }
};

REGISTER_OSGPLUGIN(osgearth_feature_cdbv, CDBVFeatureSourceFactory)

