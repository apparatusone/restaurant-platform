from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from shared.dependencies.database import get_db
from ..schemas import tables as schema
from ..controllers import tables as controller

router = APIRouter(prefix="/tables", tags=["tables"])


@router.post("/", response_model=schema.Table)
def create_table(request: schema.TableCreate, db: Session = Depends(get_db)):
    return controller.create(db=db, request=request)


@router.get("/", response_model=list[schema.Table])
def get_all_tables(db: Session = Depends(get_db)):
    return controller.read_all(db)


@router.get("/with-sessions", response_model=list[schema.TableWithSession])
def get_tables_with_sessions(db: Session = Depends(get_db)):
    """Get all tables with their current session ID (for floor view)"""
    return controller.get_tables_with_session_info(db)


@router.get("/available", response_model=list[schema.Table])
def get_available_tables(db: Session = Depends(get_db)):
    return controller.get_available_tables(db)


@router.get("/occupied", response_model=list[schema.Table])
def get_occupied_tables(db: Session = Depends(get_db)):
    return controller.get_occupied_tables(db)


@router.get("/{table_id}", response_model=schema.Table)
def get_table(table_id: int, db: Session = Depends(get_db)):
    return controller.read_one(db, table_id=table_id)


@router.get("/code/{table_code}", response_model=schema.Table)
def get_table_by_code(table_code: str, db: Session = Depends(get_db)):
    return controller.read_by_code(db, table_code=table_code)


@router.put("/{table_id}", response_model=schema.Table)
def update_table(table_id: int, request: schema.TableUpdate, db: Session = Depends(get_db)):
    return controller.update(db=db, table_id=table_id, request=request)


@router.delete("/{table_id}")
def delete_table(table_id: int, db: Session = Depends(get_db)):
    return controller.delete(db=db, table_id=table_id)