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
#include "FeatureCursorCDBV"
#include <osgEarth/Version>
#include <osgEarthFeatures/OgrUtils>
#include <osgEarthFeatures/Feature>
#include <osgEarth/Registry>
#include <algorithm>

#include <ogr_api.h>
#include <ogr_core.h>
#include <ogrsf_frmts.h>

#define LC "[FeatureCursorCDBV] "

#define OGR_SCOPED_LOCK GDAL_SCOPED_LOCK

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

static __int64 _s_CDBV_FeatureID = LONG_MAX / 2;

namespace
{
    /**
     * Determine whether a point is valid or not.  Some shapefiles can have points that are ridiculously big, which are really invalid data
     * but shapefiles have no way of marking the data as invalid.  So instead we check for really large values that are indiciative of something being wrong.
     */
    inline bool isPointValid( const osg::Vec3d& v, double thresh = 1e10 )
    {
        return (!v.isNaN() && osg::absolute( v.x() ) < thresh && osg::absolute( v.y() ) < thresh && osg::absolute( v.z() ) < thresh );
    }

    /**
     * Checks to see if all points in the Geometry are valid.
     */
    inline bool isGeometryValid( Geometry* geometry )
    {        
        if (!geometry) return false;

        if (!geometry->isValid()) return false;

        for (Geometry::const_iterator i = geometry->begin(); i != geometry->end(); ++i)
        {
            if (!isPointValid( *i ))
            {
                return false;
            }
        }
        return true;
    }
}


FeatureCursorCDBV::FeatureCursorCDBV(OGRDataSourceH              dsHandle,
                                   OGRLayerH                   layerHandle,
                                   const FeatureSource*        source,
                                   const FeatureProfile*       profile,
                                   const Symbology::Query&     query,
                                   const FeatureFilterList&    filters) :
_source           ( source ),
_dsHandle         ( dsHandle ),
_layerHandle      ( layerHandle ),
_resultSetHandle  ( 0L ),
_spatialFilter    ( 0L ),
_query            ( query ),
_chunkSize        ( 500 ),
_nextHandleToQueue( 0L ),
_profile          ( profile ),
_filters          ( filters )
{
	Set_And_Read_Data();
}

void FeatureCursorCDBV::Add_Layer_to_Cursor(OGRDataSourceH dsHandle, OGRLayerH layerHandle)
{
	_dsHandle = dsHandle;
	_layerHandle = layerHandle;
	Set_And_Read_Data();
}

void FeatureCursorCDBV::Set_And_Read_Data(void)
{
#if 0
	Read_Data_Directly();
#endif
	{
		OGR_SCOPED_LOCK;

		std::string expr;
		std::string from = OGR_FD_GetName(OGR_L_GetLayerDefn(_layerHandle));


		std::string driverName = OGR_Dr_GetName(OGR_DS_GetDriver(_dsHandle));
		// Quote the layer name if it is a shapefile, so we can handle any weird filenames like those with spaces or hyphens.
		// Or quote any layers containing spaces for PostgreSQL
		if (driverName == "ESRI Shapefile" || from.find(" ") != std::string::npos)
		{
			std::string delim = "\"";
			from = delim + from + delim;
		}

		if (_query.expression().isSet())
		{
			// build the SQL: allow the Query to include either a full SQL statement or
			// just the WHERE clause.
			expr = _query.expression().value();

			// if the expression is just a where clause, expand it into a complete SQL expression.
			std::string temp = osgEarth::toLower(expr);

			if (temp.find("select") != 0)
			{
				std::stringstream buf;
				buf << "SELECT * FROM " << from << " WHERE " << expr;
				std::string bufStr;
				bufStr = buf.str();
				expr = bufStr;
			}
		}
		else
		{
			std::stringstream buf;
			buf << "SELECT * FROM " << from;
			expr = buf.str();
		}

		//Include the order by clause if it's set
		if (_query.orderby().isSet())
		{
			std::string orderby = _query.orderby().value();

			std::string temp = osgEarth::toLower(orderby);

			if (temp.find("order by") != 0)
			{
				std::stringstream buf;
				buf << "ORDER BY " << orderby;
				std::string bufStr;
				bufStr = buf.str();
				orderby = buf.str();
			}
			expr += (" " + orderby);
		}

		// if there's a spatial extent in the query, build the spatial filter:
		if (_query.bounds().isSet())
		{
			OGRGeometryH ring = OGR_G_CreateGeometry(wkbLinearRing);
			OGR_G_AddPoint(ring, _query.bounds()->xMin(), _query.bounds()->yMin(), 0);
			OGR_G_AddPoint(ring, _query.bounds()->xMin(), _query.bounds()->yMax(), 0);
			OGR_G_AddPoint(ring, _query.bounds()->xMax(), _query.bounds()->yMax(), 0);
			OGR_G_AddPoint(ring, _query.bounds()->xMax(), _query.bounds()->yMin(), 0);
			OGR_G_AddPoint(ring, _query.bounds()->xMin(), _query.bounds()->yMin(), 0);

			_spatialFilter = OGR_G_CreateGeometry(wkbPolygon);
			OGR_G_AddGeometryDirectly(_spatialFilter, ring);
			// note: "Directly" above means _spatialFilter takes ownership if ring handle
		}


		OE_DEBUG << LC << "SQL: " << expr << std::endl;
		_resultSetHandle = OGR_DS_ExecuteSQL(_dsHandle, expr.c_str(), _spatialFilter, 0L);

		if (_resultSetHandle)
		{
			OGR_L_ResetReading(_resultSetHandle);
		}
	}

	readChunk();


}

void FeatureCursorCDBV::Read_Data_Directly(void)
{
	FeatureList preProcessList;

	OGR_SCOPED_LOCK;

	OGR_L_ResetReading(_layerHandle);
	OGRFeatureH feat_handle;

	OGRLayer * thislayer = (OGRLayer *)_layerHandle;
	int totalCount = thislayer->GetFeatureCount();
	int fcount = -1;
	while ((feat_handle = OGR_L_GetNextFeature(_layerHandle)) != NULL)
	{
		++fcount;
		if (feat_handle)
		{
			osg::ref_ptr<Feature> f = OgrUtils::createFeature(feat_handle, _profile.get());
			if (f.valid() && !_source->isBlacklisted(f->getFID()))
			{
				if (isGeometryValid(f->getGeometry()))
				{
					_queue.push(f);

					if (_filters.size() > 0)
					{
						preProcessList.push_back(f.release());
					}
				}
				else
				{
					OE_DEBUG << LC << "Skipping feature with invalid geometry: " << f->getGeoJSON() << std::endl;
				}
			}
			OGR_F_Destroy(feat_handle);
		}
	}

	// preprocess the features using the filter list:
	if (preProcessList.size() > 0)
	{
		FilterContext cx;
		cx.setProfile(_profile.get());

		for (FeatureFilterList::const_iterator i = _filters.begin(); i != _filters.end(); ++i)
		{
			FeatureFilter* filter = i->get();
			cx = filter->push(preProcessList, cx);
		}
	}

}

FeatureCursorCDBV::~FeatureCursorCDBV()
{
    OGR_SCOPED_LOCK;

    if ( _nextHandleToQueue )
        OGR_F_Destroy( _nextHandleToQueue );

    if ( _resultSetHandle != _layerHandle )
        OGR_DS_ReleaseResultSet( _dsHandle, _resultSetHandle );

    if ( _spatialFilter )
        OGR_G_DestroyGeometry( _spatialFilter );

    if ( _dsHandle )
        OGRReleaseDataSource( _dsHandle );
}

bool
FeatureCursorCDBV::hasMore() const
{
    return _resultSetHandle && ( _queue.size() > 0 || _nextHandleToQueue != 0L );
}

Feature*
FeatureCursorCDBV::nextFeature()
{
    if ( !hasMore() )
        return 0L;

    if ( _queue.size() == 0 && _nextHandleToQueue )
        readChunk();

    // do this in order to hold a reference to the feature we return, so the caller
    // doesn't have to. This lets us avoid requiring the caller to use a ref_ptr when 
    // simply iterating over the cursor, making the cursor move conventient to use.
    _lastFeatureReturned = _queue.front();
    _queue.pop();

    return _lastFeatureReturned.get();
}


// reads a chunk of features into a memory cache; do this for performance
// and to avoid needing the OGR Mutex every time
void
FeatureCursorCDBV::readChunk()
{
    if ( !_resultSetHandle )
        return;
    
    FeatureList preProcessList;
    
    OGR_SCOPED_LOCK;

    if ( _nextHandleToQueue )
    {
        osg::ref_ptr<Feature> f = OgrUtils::createFeature( _nextHandleToQueue, _profile.get() );
        if ( f.valid() && !_source->isBlacklisted(f->getFID()) )
        {
			f->setFID(_s_CDBV_FeatureID);
			++_s_CDBV_FeatureID;
            if ( isGeometryValid( f->getGeometry() ) )
            {
                _queue.push( f );

                if ( _filters.size() > 0 )
                {
                    preProcessList.push_back( f.release() );
                }
            }
            else
            {
                OE_DEBUG << LC << "Skipping feature with invalid geometry: " << f->getGeoJSON() << std::endl;
            }
        }
        OGR_F_Destroy( _nextHandleToQueue );
        _nextHandleToQueue = 0L;
    }

    unsigned handlesToQueue = _chunkSize - _queue.size();
    bool resultSetEndReached = false;

    for( unsigned i=0; i<handlesToQueue; i++ )
    {
        OGRFeatureH handle = OGR_L_GetNextFeature( _resultSetHandle );
        if ( handle )
        {
            osg::ref_ptr<Feature> f = OgrUtils::createFeature( handle, _profile.get() );
            if ( f.valid() && !_source->isBlacklisted(f->getFID()) )
            {
				f->setFID(_s_CDBV_FeatureID);
				++_s_CDBV_FeatureID;
				if (isGeometryValid( f->getGeometry() ) )
                {
                    _queue.push( f );

                    if ( _filters.size() > 0 )
                    {
                        preProcessList.push_back( f.release() );
                    }
                }
                else
                {
                    OE_DEBUG << LC << "Skipping feature with invalid geometry: " << f->getGeoJSON() << std::endl;
                }
            }            
            OGR_F_Destroy( handle );
        }
        else
        {
            resultSetEndReached = true;
            break;
        }
    }

    // preprocess the features using the filter list:
    if ( preProcessList.size() > 0 )
    {
        FilterContext cx;
        cx.setProfile( _profile.get() );

        for( FeatureFilterList::const_iterator i = _filters.begin(); i != _filters.end(); ++i )
        {
            FeatureFilter* filter = i->get();
            cx = filter->push( preProcessList, cx );
        }
    }

    // read one more for "more" detection:
    if (!resultSetEndReached)
        _nextHandleToQueue = OGR_L_GetNextFeature( _resultSetHandle );
    else
        _nextHandleToQueue = 0L;

    //OE_NOTICE << "read " << _queue.size() << " features ... " << std::endl;
}

