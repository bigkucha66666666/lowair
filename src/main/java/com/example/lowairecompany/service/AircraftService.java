package com.example.lowairecompany.service;

import com.example.lowairecompany.dao.AircraftDao;
import com.example.lowairecompany.pojo.Aircraft;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.StreamSupport;

@Service
public class AircraftService implements IAircraftService {

    @Autowired
    private AircraftDao aircraftDao;

    @Override
    public void add(Aircraft aircraft) {
        aircraftDao.save(aircraft);
    }

    @Override
    public Aircraft get(Integer id) {
        return aircraftDao.findById(id).orElseThrow(() -> new IllegalArgumentException("Aircraft 不存在"));
    }

    @Override
    public Aircraft edit(Aircraft aircraft) {
        return aircraftDao.save(aircraft);
    }

    @Override
    public Aircraft delete(Integer id) {
        Aircraft existing = aircraftDao.findById(id).orElseThrow(() -> new IllegalArgumentException("Aircraft 不存在"));
        aircraftDao.deleteById(id);
        return existing;
    }
    
    @Override
    public List<String> getAllAircraftTypes() {
        Iterable<Aircraft> aircrafts = aircraftDao.findAll();
        return StreamSupport.stream(aircrafts.spliterator(), false)
                .map(Aircraft::getCraft_type)
                .collect(Collectors.toList());
    }
    
    @Override
    public List<Aircraft> getAllAircrafts() {
        Iterable<Aircraft> aircrafts = aircraftDao.findAll();
        return StreamSupport.stream(aircrafts.spliterator(), false)
                .collect(Collectors.toList());
    }
    
    @Override
    public void deleteBatch(List<Integer> ids) {
        if (ids != null && !ids.isEmpty()) {
            aircraftDao.deleteAllById(ids);
        }
    }
}