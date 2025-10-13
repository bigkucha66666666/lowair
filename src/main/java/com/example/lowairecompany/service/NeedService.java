package com.example.lowairecompany.service;

import com.example.lowairecompany.dao.NeedDao;
import com.example.lowairecompany.pojo.Need;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

@Service
public class NeedService implements INeedService {

    @Autowired
    private NeedDao needDao;

    @Override
    public void add(Need need) {
        needDao.save(need);
    }

    @Override
    public Need get(Integer id) {
        return needDao.findById(id).orElseThrow(() -> new IllegalArgumentException("Need 不存在"));
    }

    @Override
    public Need edit(Need need) {
        return needDao.save(need);
    }

    @Override
    public Need delete(Integer id) {
        Need existing = needDao.findById(id).orElseThrow(() -> new IllegalArgumentException("Need 不存在"));
        needDao.deleteById(id);
        return existing;
    }
}


