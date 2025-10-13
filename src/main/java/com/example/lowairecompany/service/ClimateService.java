package com.example.lowairecompany.service;

import com.example.lowairecompany.dao.ClimateDao;
import com.example.lowairecompany.pojo.Climate;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.StreamSupport;

@Service
public class ClimateService implements IClimateService {

    @Autowired
    private ClimateDao climateDao;

    @Override
    public void add(Climate climate) {
        climateDao.save(climate);
    }

    @Override
    public Climate get(Integer id) {
        return climateDao.findById(id).orElseThrow(() -> new IllegalArgumentException("Climate 不存在"));
    }

    @Override
    public Climate edit(Climate climate) {
        return climateDao.save(climate);
    }

    @Override
    public Climate delete(Integer id) {
        Climate existing = climateDao.findById(id).orElseThrow(() -> new IllegalArgumentException("Climate 不存在"));
        climateDao.deleteById(id);
        return existing;
    }
    
    @Override
    public List<Climate> getAllClimates() {
        Iterable<Climate> climates = climateDao.findAll();
        return StreamSupport.stream(climates.spliterator(), false)
                .collect(Collectors.toList());
    }
    
    @Override
    public void deleteBatch(List<Integer> ids) {
        if (ids != null && !ids.isEmpty()) {
            climateDao.deleteAllById(ids);
        }
    }
}
