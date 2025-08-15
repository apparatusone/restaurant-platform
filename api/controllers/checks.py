from sqlalchemy.orm import Session, joinedload
from fastapi import HTTPException, status
from datetime import datetime
from decimal import Decimal
from ..models.checks import Check, CheckStatus
from ..models.table_sessions import TableSession
from ..schemas.checks import CheckCreate, CheckUpdate


def create(db: Session, request: CheckCreate):
    # validate session for non-virtual checks
    if not request.is_virtual and request.session_id:
        session = db.query(TableSession).filter(TableSession.id == request.session_id).first()
        if not session:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"Session with id {request.session_id} not found"
            )
        
        if session.closed_at:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Cannot create check for closed session"
            )
    elif not request.is_virtual and not request.session_id:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Non-virtual checks must have a session_id"
        )
    elif request.is_virtual and request.session_id:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Virtual checks cannot have a session_id"
        )
    
    new_check = Check(
        session_id=request.session_id,
        is_virtual=request.is_virtual,
        subtotal=request.subtotal or Decimal('0.00'),
        tax_amount=request.tax_amount or Decimal('0.00'),
        tip_amount=request.tip_amount or Decimal('0.00'),
        total_amount=request.total_amount or Decimal('0.00')
    )
    
    db.add(new_check)
    db.commit()
    db.refresh(new_check)
    return new_check


def read_all(db: Session):
    return db.query(Check).options(joinedload(Check.session)).all()


def read_one(db: Session, check_id: int):
    check = db.query(Check).options(joinedload(Check.session)).filter(Check.id == check_id).first()
    if not check:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Check with id {check_id} not found"
        )
    return check


def read_by_session(db: Session, session_id: int):
    """Get all checks for a specific session"""
    return db.query(Check).filter(Check.session_id == session_id).all()


def update(db: Session, check_id: int, request: CheckUpdate):
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Check with id {check_id} not found"
        )
    
    update_data = request.model_dump(exclude_unset=True)
    for field, value in update_data.items():
        setattr(check, field, value)
    
    # timestamps
    if request.status == CheckStatus.SUBMITTED and not check.submitted_at:
        check.submitted_at = datetime.utcnow()
    elif request.status == CheckStatus.CLOSED and not check.paid_at:
        check.paid_at = datetime.utcnow()
    
    db.commit()
    db.refresh(check)
    return check


# change check status and submitted time
def submit_check(db: Session, check_id: int):
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Check with id {check_id} not found"
        )
    
    if check.status != CheckStatus.OPEN:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Check is already {check.status.value}"
        )
    
    check.status = CheckStatus.SUBMITTED
    check.submitted_at = datetime.utcnow()
    
    db.commit()
    db.refresh(check)
    return check


def process_payment(db: Session, check_id: int, tip_amount: Decimal = None):
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Check with id {check_id} not found"
        )
    
    if check.status not in [CheckStatus.SUBMITTED, CheckStatus.PAYMENT_PENDING]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Check must be submitted before payment. Current status: {check.status.value}"
        )
    
    if tip_amount is not None:
        check.tip_amount = tip_amount
        check.total_amount = check.subtotal + check.tax_amount + check.tip_amount
    
    check.status = CheckStatus.CLOSED
    check.paid_at = datetime.utcnow()
    
    db.commit()
    db.refresh(check)
    return check


def delete(db: Session, check_id: int):
    check = db.query(Check).filter(Check.id == check_id).first()
    if not check:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Check with id {check_id} not found"
        )
    
    if check.status in [CheckStatus.SUBMITTED, CheckStatus.PAYMENT_PENDING, CheckStatus.CLOSED]:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Cannot delete submitted or paid check"
        )
    
    db.delete(check)
    db.commit()
    return {"message": f"Check {check_id} deleted successfully"}


def get_open_checks(db: Session):
    """Get all open checks"""
    return db.query(Check).filter(Check.status == CheckStatus.OPEN).options(
        joinedload(Check.session)
    ).all()


def get_pending_checks(db: Session):
    """Get all checks pending payment"""
    return db.query(Check).filter(
        Check.status.in_([CheckStatus.SUBMITTED, CheckStatus.PAYMENT_PENDING])
    ).options(joinedload(Check.session)).all()


def get_virtual_checks(db: Session):
    """Get all virtual checks (for online orders)"""
    return db.query(Check).filter(Check.is_virtual == True).all()


def get_table_checks(db: Session):
    """Get all table checks (non-virtual)"""
    return db.query(Check).filter(Check.is_virtual == False).options(
        joinedload(Check.session)
    ).all()