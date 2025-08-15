#!/usr/bin/env python3
"""
Script to add test table data
"""

from api.dependencies.database import SessionLocal
from api.models.tables import Table
from api.models import model_loader


def add_test_tables():
    model_loader.index()
    
    db = SessionLocal()
    
    try:
        # Check if tables already exist
        existing_tables = db.query(Table).count()
        if existing_tables > 0:
            print(f"Tables already exist ({existing_tables} found). Skipping creation.")
            return True
        
        tables_data = [
            {
                "code": "A1",
                "capacity": 4,
                "section": "Main Dining",
                "is_outdoor": False,
                "notes": "Near entrance",
            },
            {
                "code": "B2",
                "capacity": 2,
                "section": "Window Side",
                "is_outdoor": False,
                "notes": "Cozy for couples",
            },
            {
                "code": "C1",
                "capacity": 6,
                "section": "Patio",
                "is_outdoor": False,
                "notes": "Family table",
            },
            {
                "code": "A2",
                "capacity": 8,
                "section": "Main Dining",
                "is_outdoor": True,
                "notes": "Outdoor private",
            },
            {
                "code": "A3",
                "capacity": 8,
                "section": "Main Dining",
                "is_outdoor": True,
                "notes": "Near entrance",
            },
            {
                "code": "A4",
                "capacity": 8,
                "section": "Main Dining",
                "is_outdoor": True,
                "notes": "Near entrance",
            }
        ]
        
        created_tables = []
        
        for table_data in tables_data:
            table = Table(
                code=table_data["code"],
                capacity=table_data["capacity"],
                section=table_data["section"],
                is_outdoor=table_data["is_outdoor"],
                notes=table_data["notes"]
            )
            
            db.add(table)
            db.flush()
            
            created_tables.append({
                "id": table.id,
                "code": table.code,
                "capacity": table.capacity,
                "section": table.section,
                "is_outdoor": table.is_outdoor,
                "notes": table.notes
            })
        
        db.commit()
        print(f"Successfully created {len(created_tables)} test tables")
        for table in created_tables:
            print(f"  - {table['code']}: {table['capacity']} seats in {table['section']} ({'Outdoor' if table['is_outdoor'] else 'Indoor'})")
        return True
        
    except Exception as e:
        db.rollback()
        print(f"Error adding test tables: {e}")
        return False
    
    finally:
        db.close()


if __name__ == "__main__":
    add_test_tables()