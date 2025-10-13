package com.example.lowairecompany.dao;

import com.example.lowairecompany.pojo.Aircraft;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface AircraftDao extends CrudRepository<Aircraft, Integer> {
}


