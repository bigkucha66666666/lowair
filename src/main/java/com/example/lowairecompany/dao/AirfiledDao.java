package com.example.lowairecompany.dao;

import com.example.lowairecompany.pojo.Airfiled;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface AirfiledDao extends CrudRepository<Airfiled, Integer> {
}


