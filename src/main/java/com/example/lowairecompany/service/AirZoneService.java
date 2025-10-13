package com.example.lowairecompany.service;

import com.example.lowairecompany.dao.AirZoneDao;
import com.example.lowairecompany.pojo.AirZone;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.StreamSupport;

@Service
public class AirZoneService implements IAirZoneService {

    @Autowired
    private AirZoneDao airZoneDao;

    @Override
    public void add(AirZone airZone) {
        airZoneDao.save(airZone);
    }

    @Override
    public AirZone get(Integer zoneId) {
        return airZoneDao.findById(zoneId).orElseThrow(() -> new IllegalArgumentException("AirZone 不存在"));
    }

    @Override
    public AirZone edit(AirZone airZone) {
        return airZoneDao.save(airZone);
    }

    @Override
    public AirZone delete(Integer zoneId) {
        AirZone existing = airZoneDao.findById(zoneId).orElseThrow(() -> new IllegalArgumentException("AirZone 不存在"));
        airZoneDao.deleteById(zoneId);
        return existing;
    }
    
    @Override
    public List<String> getAllZoneNames() {
        Iterable<AirZone> airZones = airZoneDao.findAll();
        return StreamSupport.stream(airZones.spliterator(), false)
                .map(AirZone::getZoneName)
                .collect(Collectors.toList());
    }
    
    @Override
    public List<AirZone> getAllZones() {
        Iterable<AirZone> airZones = airZoneDao.findAll();
        return StreamSupport.stream(airZones.spliterator(), false)
                .collect(Collectors.toList());
    }
    
    @Override
    public void deleteBatch(List<Integer> zoneIds) {
        if (zoneIds != null && !zoneIds.isEmpty()) {
            airZoneDao.deleteAllById(zoneIds);
        }
    }
}
