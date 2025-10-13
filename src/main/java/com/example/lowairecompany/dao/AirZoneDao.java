package com.example.lowairecompany.dao;

import com.example.lowairecompany.pojo.AirZone;
import org.springframework.data.repository.CrudRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface AirZoneDao extends CrudRepository<AirZone, Integer> {
}
