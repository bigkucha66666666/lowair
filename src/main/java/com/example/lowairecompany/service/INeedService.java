package com.example.lowairecompany.service;

import com.example.lowairecompany.pojo.Need;

public interface INeedService {
    void add(Need need);
    Need get(Integer id);
    Need edit(Need need);
    Need delete(Integer id);
}


