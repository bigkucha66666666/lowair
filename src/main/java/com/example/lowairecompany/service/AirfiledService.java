package com.example.lowairecompany.service;

import com.example.lowairecompany.dao.AirfiledDao;
import com.example.lowairecompany.pojo.Airfiled;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.StreamSupport;

@Service
public class AirfiledService implements IAirfiledService {

    @Autowired
    private AirfiledDao airfiledDao;

    @Override
    public void add(Airfiled airfiled) {
        airfiledDao.save(airfiled);
    }

    @Override
    public Airfiled get(Integer id) {
        return airfiledDao.findById(id).orElseThrow(() -> new IllegalArgumentException("Airfiled 不存在"));
    }

    @Override
    public Airfiled edit(Airfiled airfiled) {
        return airfiledDao.save(airfiled);
    }

    @Override
    public Airfiled delete(Integer id) {
        Airfiled existing = airfiledDao.findById(id).orElseThrow(() -> new IllegalArgumentException("Airfiled 不存在"));
        airfiledDao.deleteById(id);
        return existing;
    }
    
    @Override
    public List<String> getAllAirfiledNames() {
        Iterable<Airfiled> airfileds = airfiledDao.findAll();
        return StreamSupport.stream(airfileds.spliterator(), false)
                .map(Airfiled::getName)
                .collect(Collectors.toList());
    }
    
    @Override
    public List<Airfiled> getAllAirfileds() {
        Iterable<Airfiled> airfileds = airfiledDao.findAll();
        return StreamSupport.stream(airfileds.spliterator(), false)
                .collect(Collectors.toList());
    }
    
    @Override
    public void deleteBatch(List<Integer> ids) {
        if (ids != null && !ids.isEmpty()) {
            airfiledDao.deleteAllById(ids);
        }
    }
}